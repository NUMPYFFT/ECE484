import time
import math
import numpy as np
import cv2
import rospy
import matplotlib.pyplot as plt

from line_fit import line_fit, tune_fit, bird_fit, final_viz
from Line import Line
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from skimage import morphology



class lanenet_detector():
    def __init__(self):

        self.bridge = CvBridge()
        # NOTE
        # Uncomment this line for lane detection of GEM car in Gazebo
        self.sub_image = rospy.Subscriber('/gem/front_single_camera/front_single_camera/image_raw', Image, self.img_callback, queue_size=1)
        # Uncomment this line for lane detection of videos in rosbag
        # self.sub_image = rospy.Subscriber('camera/image_raw', Image, self.img_callback, queue_size=1)
        self.pub_image = rospy.Publisher("lane_detection/annotate", Image, queue_size=1)
        self.pub_bird = rospy.Publisher("lane_detection/birdseye", Image, queue_size=1)
        self.left_line = Line(n=5)
        self.right_line = Line(n=5)
        self.detected = False
        self.hist = True


    def img_callback(self, data):

        try:
            # Convert a ROS image message into an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        raw_img = cv_image.copy()
        mask_image, bird_image = self.detection(raw_img)

        if mask_image is not None and bird_image is not None:
            # Convert an OpenCV image into a ROS image message
            out_img_msg = self.bridge.cv2_to_imgmsg(mask_image, 'bgr8')
            out_bird_msg = self.bridge.cv2_to_imgmsg(bird_image, 'bgr8')

            # Publish image message in ROS
            self.pub_image.publish(out_img_msg)
            self.pub_bird.publish(out_bird_msg)


    def gradient_thresh(self, img, thresh_min=25, thresh_max=100):
        """
        Apply sobel edge detection on input image in x, y direction
        """
        #1. Convert the image to gray scale
        #2. Gaussian blur the image
        #3. Use cv2.Sobel() to find derievatives for both X and Y Axis
        #4. Use cv2.addWeighted() to combine the results
        #5. Convert each pixel to unint8, then apply threshold to get binary image

        ## TODO

        ####
        scale = 1
        delta = 0
        ddepth = cv2.CV_16S

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (3,3), 0)
        grad_x = cv2.Sobel(blur, ddepth, 1, 0, ksize=3, scale=scale, delta=delta, borderType=cv2.BORDER_DEFAULT)
        grad_y = cv2.Sobel(blur, ddepth, 0, 1, ksize=3, scale=scale, delta=delta, borderType=cv2.BORDER_DEFAULT)

        abs_grad_x = cv2.convertScaleAbs(grad_x)
        abs_grad_y = cv2.convertScaleAbs(grad_y)
        grad = cv2.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
        # binary_output = np.zeros((img.shape))
        binary_output = np.where(((grad >= thresh_min) & (grad <= thresh_max)), 1, 0)
        cv2.imwrite("/home/cl78/Desktop/mp1_img/grad.jpg", 255*binary_output)
        
        return binary_output



    
    def color_thresh(self, img, thresh=(100, 255)):
        """
        Convert RGB to HSL and threshold to binary image using S channel
        """
        #1. Convert the image from RGB to HSL
        #2. Apply threshold on S channel to get binary image
        #Hint: threshold on H to remove green grass
        ## TODO
    

        
        ####
        #img[:, :, 1] = 0
        yellow_lower = (0,0,50)
        yellow_upper = (38,255,255)
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #mask_green = cv2.inRange(hsv_img, (36, 0, 0), (70, 255,255))
        #green_filtered = 255 - mask_green
        mask = cv2.inRange(hsv_img, yellow_lower, yellow_upper) # <lower, >upper => 0

        #mask_img = cv2.bitwise_and(img,img, mask=mask)
        binary_output = mask
        
        '''
        gray_img =cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        white_binary = np.zeros_like(gray_img)
        white_binary[(gray_img > 200) & (gray_img <= 255)] = 1

        # Convert image to HLS
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        H = hls[:,:,0]
        S = hls[:,:,2]
        sat_binary = np.zeros_like(S)
        # Detect pixels that have a high saturation value
        sat_binary[(S > 90) & (S <= 255)] = 1

        hue_binary =  np.zeros_like(H)
        # Detect pixels that are yellow using the hue component
        hue_binary[(H > 10) & (H <= 25)] = 1

        # Combine all pixels detected above
        binary_2 = cv2.bitwise_or(hue_binary, sat_binary)
        binary_output = binary_2
        '''

        cv2.imwrite("/home/cl78/Desktop/mp1_img/orig.jpg", 255*img)
        cv2.imwrite("/home/cl78/Desktop/mp1_img/color.jpg", binary_output)
        return binary_output
        

    def binary_thresholded(self, img):
        # Transform image to gray scale
        gray_img =cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Apply sobel (derivative) in x direction, this is usefull to detect lines that tend to be vertical
        sobelx = cv2.Sobel(gray_img, cv2.CV_64F, 1, 0)
        abs_sobelx = np.absolute(sobelx)
        # Scale result to 0-255
        scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
        sx_binary = np.zeros_like(scaled_sobel)
        # Keep only derivative values that are in the margin of interest
        sx_binary[(scaled_sobel >= 30) & (scaled_sobel <= 255)] = 1

        # Detect pixels that are white in the grayscale image
        white_binary = np.zeros_like(gray_img)
        white_binary[(gray_img > 200) & (gray_img <= 255)] = 1

        # Convert image to HLS
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        H = hls[:,:,0]
        S = hls[:,:,2]
        sat_binary = np.zeros_like(S)
        # Detect pixels that have a high saturation value
        sat_binary[(S > 90) & (S <= 255)] = 1

        hue_binary =  np.zeros_like(H)
        # Detect pixels that are yellow using the hue component
        hue_binary[(H > 10) & (H <= 25)] = 1

        # Combine all pixels detected above
        binary_1 = cv2.bitwise_or(sx_binary, white_binary)
        binary_2 = cv2.bitwise_or(hue_binary, sat_binary)
        binary = cv2.bitwise_or(binary_1, binary_2)
        #plt.imshow(binary, cmap='gray')

        #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # Draw figure for binary images
        #f, axarr = plt.subplots(1,6)
        #f.set_size_inches(25, 8)
        #axarr[0].imshow(img)
        #axarr[1].imshow(sx_binary, cmap='gray')
        #axarr[2].imshow(white_binary, cmap='gray')
        #axarr[3].imshow(sat_binary, cmap='gray')
        #axarr[4].imshow(hue_binary, cmap='gray')
        #axarr[5].imshow(binary, cmap='gray')
        #axarr[0].set_title("Undistorted Image")
        #axarr[1].set_title("x Sobel Derivative")
        #axarr[2].set_title("White Threshold")
        #axarr[3].set_title("Saturation Threshold")
        #axarr[4].set_title("Hue Threshold")
        #axarr[5].set_title("Combined")
        #axarr[0].axis('off')
        #axarr[1].axis('off')
        #axarr[2].axis('off')
        #axarr[3].axis('off')
        #axarr[4].axis('off')
        #axarr[5].axis('off')
    
        return binary

    #out_img = binary_thresholded(img)

    def combinedBinaryImage(self, img):
        """
        Get combined binary image from color filter and sobel filter
        """
        #1. Apply sobel filter and color filter on input image
        #2. Combine the outputs
        ## Here you can use as many methods as you want.

        ## TODO

        ####
        SobelOutput = self.gradient_thresh(img, thresh_min=25, thresh_max=100)
        ColorOutput = self.color_thresh(img, thresh=(100, 255))/255
        binaryImage = np.zeros_like(SobelOutput)
        binaryImage[(ColorOutput==1)|(SobelOutput==1)] = 1
        # Remove noise from binary image
        binaryImage = morphology.remove_small_objects(binaryImage.astype('bool'),min_size=50,connectivity=2)
        cv2.imwrite("/home/cl78/Desktop/mp1_img/combined.jpg", 255*binaryImage)
        cv2.imwrite("/home/cl78/Desktop/mp1_img/sobel.jpg", 255*SobelOutput)
        cv2.imwrite("/home/cl78/Desktop/mp1_img/colorout.jpg", ColorOutput)

        return binaryImage


    def perspective_transform(self, img, verbose=False):
        """
        Get bird's eye view from input image
        """
        #1. Visually determine 4 source points and 4 destination points
        #2. Get M, the transform matrix, and Minv, the inverse using cv2.getPerspectiveTransform()
        #3. Generate warped image in bird view using cv2.warpPerspective()

        ## TOD

        ####
        pt_A = [255, 280]
        pt_B = [395, 280]
        pt_C = [590, 390]
        pt_D = [70, 390]
        
        input_pts = np.float32([pt_A, pt_B, pt_C, pt_D])
        output_pts = np.float32([[40, 40], [440, 40], [440, 600], [40, 600]])

        img_2 = img
        img_2 = cv2.circle(np.float32(img_2), (255,280), 3, (255, 0, 0), 2)
        img_2 = cv2.circle(np.float32(img_2), (395,280), 3, (255, 0, 0), 2)
        img_2 = cv2.circle(np.float32(img_2), (590,390), 3, (255, 0, 0), 2)
        img_2 = cv2.circle(np.float32(img_2), (70,390), 3, (255, 0, 0), 2)
        M = cv2.getPerspectiveTransform(input_pts, output_pts)
        Minv = np.linalg.inv(M)
        warped_img = cv2.warpPerspective(np.float32(img), M, (img.shape))
        cv2.imwrite("/home/cl78/Desktop/mp1_img/perspective.jpg", 255*warped_img)
        cv2.imwrite("/home/cl78/Desktop/mp1_img/inputimg.jpg", 255*img_2)

        return warped_img, M, Minv


    def detection(self, img):

        binary_img = self.combinedBinaryImage(img)
        # binary_img = self.binary_thresholded(img)
        img_birdeye, M, Minv = self.perspective_transform(binary_img)
        cv2.imwrite("/home/cl78/Desktop/mp1_img/birdimg.jpg", 255*img_birdeye)

        # print(self.hist)
        if not self.hist:
            # Fit lane without previous result
            ret = line_fit(img_birdeye)
            print("1")
            left_fit = ret['left_fit']
            right_fit = ret['right_fit']
            nonzerox = ret['nonzerox']
            nonzeroy = ret['nonzeroy']
            left_lane_inds = ret['left_lane_inds']
            right_lane_inds = ret['right_lane_inds']

        else:
            # Fit lane with previous result
            if not self.detected:
                ret = line_fit(img_birdeye)
                print("2")
                if ret is not None:
                    print("2gg")
                    #print(ret)
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                    self.detected = True

            else:
                left_fit = self.left_line.get_fit()
                right_fit = self.right_line.get_fit()
                ret = tune_fit(img_birdeye, left_fit, right_fit)
                print("3")

                if ret is not None:
                    print("3gg")
                    #print(ret)
                    left_fit = ret['left_fit']
                    right_fit = ret['right_fit']
                    nonzerox = ret['nonzerox']
                    nonzeroy = ret['nonzeroy']
                    left_lane_inds = ret['left_lane_inds']
                    right_lane_inds = ret['right_lane_inds']

                    left_fit = self.left_line.add_fit(left_fit)
                    right_fit = self.right_line.add_fit(right_fit)

                else:
                    print("3n")
                    self.detected = False

            # Annotate original image
            bird_fit_img = None
            combine_fit_img = None
            if ret is not None:
                bird_fit_img = bird_fit(img_birdeye, ret, save_file=None)
                combine_fit_img = final_viz(img, left_fit, right_fit, Minv)
                cv2.imwrite("/home/cl78/Desktop/mp1_img/img_f.jpg", 255*combine_fit_img)
            else:
                print("11111Unable to detect lanes")

            return combine_fit_img, bird_fit_img


if __name__ == '__main__':
    # init args
    rospy.init_node('lanenet_node', anonymous=True)
    lanenet_detector()
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
