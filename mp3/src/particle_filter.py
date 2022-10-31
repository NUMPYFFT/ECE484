from cmath import sqrt
import numpy as np
from maze import Maze, Particle, Robot
import bisect
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode
import controller
import copy
import matplotlib.pyplot as plt 

def vehicle_dynamics(t, vars, vr, delta):
    curr_x = vars[0]
    curr_y = vars[1] 
    curr_theta = vars[2]
    
    dx = vr * np.cos(curr_theta)
    dy = vr * np.sin(curr_theta)
    dtheta = delta
    return np.array([dx,dy,dtheta])

class particleFilter:
    def __init__(self, bob, world, num_particles, sensor_limit, x_start, y_start):
        self.num_particles = num_particles  # The number of particles for the particle filter
        self.sensor_limit = sensor_limit    # The sensor limit of the sensor
        particles = list()

        for i in range(num_particles):
            x = np.random.uniform(0, world.width)
            y = np.random.uniform(0, world.height)
            particles.append(Particle(x = x, y = y, maze = world, sensor_limit = sensor_limit))
        self.particles = particles          # Randomly assign particles at the begining
        self.bob = bob                      # The estimated robot state
        self.world = world                  # The map of the maze
        self.x_start = x_start              # The starting position of the map in the gazebo simulator
        self.y_start = y_start              # The starting position of the map in the gazebo simulator
        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.controlSub = rospy.Subscriber("/gem/control", Float32MultiArray, self.__controlHandler, queue_size = 1)
        self.control = []                   # A list of control signal from the vehicle
        # print(self.control)
        return

    def __controlHandler(self,data):
        """
        Description:
            Subscriber callback for /gem/control. Store control input from gem controller to be used in particleMotionModel.
        """
        tmp = list(data.data)
        self.control.append(tmp)

    def getModelState(self):
        """
        Description:
            Requests the current state of the polaris model when called
        Returns:
            modelState: contains the current model state of the polaris vehicle in gazebo
        """

        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='polaris')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState

    def weight_gaussian_kernel(self,x1, x2, std = 5000):
        tmp1 = np.array(x1)
        tmp2 = np.array(x2)
        return np.sum(np.exp(-((tmp2-tmp1) ** 2) / (2 * std)))

    def updateWeight(self, readings_robot):
        """
        Description:
            Update the weight of each particles according to the sensor reading from the robot 
        Input:
            readings_robot: List, contains the distance between robot and wall in [front, right, rear, left] direction.
        """

        ## TODO #####
        weights = []
        for i in range(len(self.particles)):
            weights.append(self.weight_gaussian_kernel(readings_robot, self.particles[i].read_sensor(), std = 10000))
        weights = weights / sum(weights)
        print(min(weights), max(weights))
        for i in range(len(self.particles)):
            self.particles[i].weight = weights[i]
        ###############
        pass

    def resampleParticle(self):
        """
        Description:
            Perform resample to get a new list of particles 
        """
        particles_new = list()

        ## TODO #####

        N = len(self.particles)
        

        cum_sum = np.zeros(N)
        sum_n = 0
        for i in range(N):
            sum_n += self.particles[i].weight
            cum_sum[i] = sum_n
        #print(cum_sum[-1])
        
        for i in range(N):
            num = np.random.random(size=1)
            # idx = np.argmin(abs(cum_sum - num))
            for j in range (1, N):
                if (num <= cum_sum[0]):
                    idx = 0
                    break
                elif (num > cum_sum[j-1] and num <= cum_sum[j]):
                    idx = j
                    break
                else:
                    idx = 0
            # print(idx)
            particles_new.append(Particle(x = self.particles[idx].x, y = self.particles[idx].y, heading = self.particles[idx].heading, weight = self.particles[idx].weight, maze = self.particles[idx].maze, sensor_limit = self.sensor_limit, noisy = True))
        self.particles = particles_new
        assert (N == len(particles_new))
        ###############

        self.particles = particles_new

    def particleMotionModel(self):
        """
        Description:
            Estimate the next state for each particle according to the control input from actual robot 
        """
        ## TODO #####

        control_signals = copy.deepcopy(self.control)
        
        '''
        #print(control_signal)
        while (len(control_signal) <= len(self.partglobal lastcontrolicles)):
            a=1
        
        for i in range (len(self.particles)):
        
            #control_signal = self.control[i]
            #print(control_signal)
            #print(i,"a")
            #control_signal = self.control[i] #velocity and delta
            x = self.particles[i].x
            y = self.particles[i].y
            theta = self.particles[i].heading
            currentPose = np.array([x, y, theta])
            parameters = vehicle_dynamics(0.01, currentPose, control_signal[i][0], control_signal[i][1]) #get dx/dt, dy/dt, dtheta/dt
            self.particles[i].x += 0.01 * np.cos(particles[i].heading) * control_signal[i][0]
            self.particles[i].y += 0.01 * np.sin(particles[i].heading) * control_signal[i][0]
            self.particles[i].heading += 0.01 * control_signals[i][1]
            
        print("c")
        '''
        #for i in range(len(self.particles)):
         #   for cnt in control_signal:

        temp = []
        #for i, cnt in enumerate(control_signal):
        for i in range(self.lastcontrol,len(control_signals)):
        #for control in control_signals:
            # print("length of signals: ", len(control_signals))
            for j in range(len(self.particles)):
                x = self.particles[j].x
                y = self.particles[j].y
                theta = self.particles[j].heading
                #x = self.bob.x
                #y = self.bob.y
                #theta = self.bob.heading

                #currentPose = np.array([x, y, theta])
                #parameters = vehicle_dynamics(0.01, currentPose, control_signals[i][0], control_signals[i][1]) * 0.01 #get dx/dt, dy/dt, dtheta/dt
                #parameters = vehicle_dynamics(0.01, currentPose, control_signals[i][0], control_signals[i][1]) * 0.01 #get dx/dt, dy/dt, dtheta/dt
                # self.particles[j].try_move(parameters, self.particles[j].maze, noisy = False)
            
                self.particles[j].x += 0.01 * np.cos(self.particles[j].heading) * control_signals[i][0]
                self.particles[j].y += 0.01 * np.sin(self.particles[j].heading) * control_signals[i][0]
                self.particles[j].heading += 0.01 * control_signals[i][1]

                #if self.particles[j].try_move(parameters, self.particles[j].maze, noisy = False) == False:
                #    self.particles[j].fix_invalid_particles()
                temp.append([self.particles[j].x, self.particles[j].y, self.particles[j].heading])
        
        #print("number of unique particles: ", len(np.unique(np.array(temp))))


        self.lastcontrol = len(control_signals)
        

           

        ###############
        pass


    lastcontrol = 0
    def runFilter(self):
        """
        Description:
            Run PF localization
        """
        # global lastcontrol
        
        i = 0
        err_dist = []
        err_orientation = []
        iters = []
        while i <= 150:
            print("number of iterations: ", i)
            i += 1
            ## TODO #####
            # Finish this function to have the particle filter running
            
            # Read sensor msg
            #if (len(self.control) != 0):
            self.particleMotionModel()
                
            reading = self.bob.read_sensor()
            
            self.updateWeight(reading)
            self.resampleParticle()

            # Display robot and particles on map
            assert(len(self.particles)==1000)
            
            #print(len(np.unique(self.particles.x)))
            self.world.show_particles(particles = self.particles, show_frequency = 10)
            
            self.world.show_robot(robot = self.bob)
            
            [est_x,est_y,est_heading] = self.world.show_estimated_location(particles = self.particles)

            err_dist.append(sqrt((est_x - self.bob.x)**2 + (est_y - self.bob.y)**2))
            err_orientation.append(abs(est_heading - self.bob.heading*180/np.pi))

            iters.append(i)
        
            self.world.clear_objects()
        plt.figure(figsize=(8,8))

        plt.subplot(1,2,1)
        plt.plot(iters, err_dist)
        plt.title("error in position vs iterations")
        plt.xlabel("iterations")
        plt.ylabel("error in position")
        plt.tight_layout()
        plt.subplot(1,2,2)
        plt.plot(iters, err_orientation)
        plt.title("error in orientation(degrees) vs iterations")
        plt.xlabel("iterations")
        plt.ylabel("error in orientation")
        plt.tight_layout()

        plt.savefig("8 directions")
        err_dist_avg = np.sum(np.array(err_dist)) / 150
        err_orientation_avg = np.sum(np.array(err_orientation)) / 150
        print("average error of distance:", err_dist_avg)
        print("average error of orientation:", err_orientation_avg)
            ###############
