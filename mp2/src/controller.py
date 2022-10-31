import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
from util import euler_to_quaternion, quaternion_to_euler

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)

    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp

    def execute(self, currentPose, referencePose):
        # Compute the control input to the vehicle according to the 
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   referencePose: list, the reference state of the vehicle, 
        #       the element in the list are [ref_x, ref_y, ref_theta, ref_v]
        # Output: None

        # TODO: Implement this function
        # print(currentPose)
        # print(referencePose)
        # print(currentPose)
        x_ref, y_ref, theta_ref, v_ref = referencePose
        x_b = currentPose.pose.position.x
        y_b = currentPose.pose.position.y
        theta_b = quaternion_to_euler(currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z, currentPose.pose.orientation.w)[2]
        v_b = np.sqrt(currentPose.twist.linear.x**2+currentPose.twist.linear.y**2)
       
        #v_b = currentPose.twist.linear.x
        #print(v_b)

        delta_x = np.cos(theta_ref) * (x_ref - x_b) + np.sin(theta_ref) * (y_ref - y_b)
        delta_y = -np.sin(theta_ref) * (x_ref - x_b) + np.cos(theta_ref) * (y_ref - y_b)
        delta_theta = theta_ref - theta_b
        delta_v = v_ref - v_b
        # print(v_ref)

        k_x, k_y, k_v, k_theta = 0.2, 0.2, 1, 1
        K = np.array([[k_x, 0, 0, k_v],
                      [0, k_y, k_theta, 0]])

        delta = np.array([[delta_x],
                          [delta_y],
                          [delta_theta],
                          [delta_v]])
        # print(K.shape, delta.shape)
        u = K @ delta
        #print(u)
        #Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive(steering_angle=u[1], speed=u[0])
        #newAckermannCmd = AckermannDrive(steering_angle=0, speed=10)
        #newAckermannCmd = AckermannDrive(steering_angle=0, steering_angle_velocity=0,speed=0,acceleration=0, jerk=0)


        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)


    def setModelState(self, currState, targetState, vehicle_state = "run"):
        control = self.rearWheelFeedback(currState, targetState)
        self.controlPub.publish(control)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)