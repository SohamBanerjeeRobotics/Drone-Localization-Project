import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

#### Global Parameters for plotting ####
X_array = [] # Initializing X array
Y_array = [] # Initializing Y array
########################################
#### Global Variable for convergence check.
Source_Temperature_Threshold = 200.0 # user defined based on experiments.
Epsilon = 0.3 
#### Defining some global functions ######
def temperature_distribution_plot(x_pos_drone, y_pos_drone, T0 = 0.0, SX = 3.5, SY = 3.0, ST = 900.0, Z = 1.0, DTMax = 200.0, LT = 2.0, KH = 0.2):
    # Position of drone w.r.t fire source
    R_drone_fire = np.sqrt(((x_pos_drone - SX) ** 2) + ((y_pos_drone - SY) ** 2))
    A_param = (5.386 * (((Z / LT) + 0.558) ** 3.205)) * np.exp((-4.057) * (Z / LT))
    T_drone = T0 + (DTMax * (np.log(100) / np.log(1 + ST)) * np.exp((-KH) * R_drone_fire) * A_param)
    return T_drone
  
  
def temperature(r_drone, theta_drone, T0 = 0.0, SX = 3.5, SY = 3.0, ST = 900.0, Z = 1.0, DTMax = 200.0, LT = 2.0, KH = 0.2):
    x_pos_drone = r_drone * np.cos(theta_drone)
    y_pos_drone = r_drone * np.sin(theta_drone)
  
    # Position of drone w.r.t fire source
    R_drone_fire = np.sqrt(((x_pos_drone - SX) ** 2) + ((y_pos_drone - SY) ** 2))
    A_param = (5.386 * (((Z / LT) + 0.558) ** 3.205)) * np.exp((-4.057) * (Z / LT))
    T_drone = T0 + (DTMax * (np.log(100) / np.log(1 + ST)) * np.exp((-KH) * R_drone_fire) * A_param)
    return T_drone


def temperature_gradient(current_drone_temp, previous_drone_temp, r_drone):
    grad_drone = (current_drone_temp - previous_drone_temp) / r_drone
    return grad_drone
##########################################



class TargetPublisherNode(Node):
    def __init__(self):
        super().__init__("TargetPublisherNodeGreedy")

        self.create_subscription(PointStamped, "/Crazyflie/gps", self.gps_callback, 10)
        self.pub = self.create_publisher(Point, "/target_pos", 10)


        self.current_gps_drone = None
        self.target_pos_drone = None
        self.theta_drone_step = None
        self.r_drone_step = 0.1 # Initial step size.
        
        ### Flags for reinitializing random walk
        self.r_drone_step_previous = 0.0
        self.theta_drone_step_previous = 0.0
        
        ### variables to check convergence.
        self.temp_grad_array_values = []
        self.temp_r_theata_array = []
        self.temp_value_array = []
        self.temp_value_previous = None
        self.convergence_count = 0
        
        ### variables updated at each publication ###
        self.temp_grad_array = np.zeros(72) # creates an array to store grad data
        self.pub_step = 0 # initializing publishing step


    def gps_callback(self, msg: PointStamped):
        self.current_gps_drone = (msg.point.x, msg.point.y) # storing (x,y) coordinates.
        self.greedy() # Calls the greedy function.


    def greedy(self):
        if self.pub_step != 0:
            distance_remaining = np.linalg.norm(np.array(self.target_pos_drone) - np.array(self.current_gps_drone))
            # To check whether the drone has reached target position or not
            if distance_remaining <= 0.02:
                print(f"Reached : {self.target_pos_drone}")
                temperature_drone_r_x = (self.r_drone_step_previous * np.cos(self.theta_drone_step_previous))
                                                + (self.r_drone_step * np.cos(self.theta_drone_step))
                temperature_drone_r_y = (self.r_drone_Step_previous * np.sin(self.theta_drone_step_previous)) 
                                                + (self.r_drone_step * np.sin(self.theta_drone_step))
                temperature_drone_r = np.sqrt((temperature_drone_r_x ** 2) + (temperature_drone_r_y ** 2))
                temperature_drone_theta = np.atan(temperature_drone_r_y / temperature_drone_r_x)    
                temperature_drone = temperature(temperature_drone_r, temperature_drone_theta)
                if self.pub_step <= 12:
                    temperature_drone_previous = temperature(self.r_drone_step_previous, self.theta_drone_step_previous)
                    temperature_gradient_drone = temperature_gradient(temperature_drone, temperature_drone_previous, self.r_drone_step)
                    self.temp_grad_array_values.append(temperature_gradient_drone)
                    
                    self.temp_grad_array = self.temp_grad_array + (temperature_gradient_drone * np.cos(self.theta_drone_step - np.arange(0, 2 * np.pi, (5 * np.pi) / 180)))
                # Convergence purpose
                if.self.pub_step >= 12:
                    if self.pub_step == 12:
                        self.temp_r_theta_array.append((self.r_drone_step_previous, self.theta_drone_step_previous))
                        self.temp_value_array.append(temperature(self.r_drone_step_previous, self.theta_drone_step_previous))
                    else:
                        self.temp_r_theta_array.append((temperature_drone_r, temperature_drone_theta))
                        self.temp_value_array.append(temperature_drone)
                    self.Check_Convergence()
                
                
                # Code to reinitialize random walk
                if self.pub_step == 17:
                    r_drone_step_previous_x = (self.r_drone_step_previous * np.cos(self.theta_drone_step_previous)) 
                                                + (self.r_drone_step * np.cos(self.theta_drone_step))
                    r_drone_step_previous_y = (self.r_drone_Step_previous * np.sin(self.theta_drone_step_previous)) 
                                                + (self.r_drone_step * np.sin(self.theta_drone_step))
                    self.r_drone_step_previous = np.sqrt((r_drone_step_previous_x ** 2) + (r_drone_step_previous_y ** 2))
                    self.theta_drone_step_previous = np.atan(r_drone_step_previous_y / r_drone_step_previous_x)
                    # It makes the random walk to happen.
                    self.pub_step = 0 
                    self.r_drone_step = 0.1
                    self.theta_drone_step = None
                    self.temp_grad_array = np.zeros(72)
                    # Convergence purpose
                    self.convergence_count = 0
                    self.temp_r_theta_array = []
                    self.temp_value_array = []
                    return # break out of the function.
                
                
                if self.pub_step  >= 12: # specify no. of random steps to take
                    self.r_drone_step = self.r_drone_step + 0.25 # step increment between two random walks.
                    # Epsilon Greedy
                    if np.random.random() < Epsilon:
                        # Exploration Step
                        random_index = np.random.randint(0, len(self.temp_grad_array))
                        self.theta_drone_step = random_index * ((5 * np.pi) / 180)
                    else:
                        # Exploitation Step
                        self.theta_drone_step = np.argmax(self.temp_grad_array) * ((5 * np.pi) / 180)
          
                    # Generalized drone target position
                    x_target_drone = (self.r_drone_step_previous * np.cos(self.theta_drone_step_previous)) 
                                         + (self.r_drone_step * np.cos(self.theta_drone_step))
                    y_target_drone = (self.r_drone_Step_previous * np.sin(self.theta_drone_step_previous)) 
                                         + (self.r_drone_step * np.sin(self.theta_drone_step))
                    self.target_pos_drone = (x_target_drone, y_target_drone)

                    ## publishing target position
                    publish_msg = Point()
                    publish_msg.x = x_target_drone
                    publish_msg.y = y_target_drone
                    publish_msg.z = 1.0

                    self.pub.publish(publish_msg)
                  
                    # Storing positions
                    X_array.append(x_target_drone)
                    Y_array.append(y_target_drone)

                    self.pub_step = self.pub_step + 1

                else:
                    # update r_drone_step value
                    self.r_drone_step = self.r_drone_step + 0.0 # This update is during random walk.
                    # choose a random theta
                    theta_drone_array = np.arange(0, 2 * np.pi, (5 * np.pi) / 180)
                    self.theta_drone_step = np.random.choice(theta_drone_array) 

                    # Generalized drone target position
                    x_target_drone = (self.r_drone_step_previous * np.cos(self.theta_drone_step_previous)) 
                                         + (self.r_drone_step * np.cos(self.theta_drone_step))
                    y_target_drone = (self.r_drone_Step_previous * np.sin(self.theta_drone_step_previous)) 
                                         + (self.r_drone_step * np.sin(self.theta_drone_step))
                    self.target_pos_drone = (x_target_drone, y_target_drone)

                    ## publishing target position
                    publish_msg = Point()
                    publish_msg.x = x_target_drone
                    publish_msg.y = y_target_drone
                    publish_msg.z = 1.0

                    self.pub.publish(publish_msg)

                    # Storing positions
                    X_array.append(x_target_drone) 
                    Y_array.append(y_target_drone)

                    self.pub_step = self.pub_step + 1
        else:
            # choose a random theta
            theta_drone_array = np.arange(0, 2 * np.pi, (5 * np.pi) / 180)
            self.theta_drone_step = np.random.choice(theta_drone_array) 

            # Generalized drone target position
            x_target_drone = (self.r_drone_step_previous * np.cos(self.theta_drone_step_previous)) 
                                    + (self.r_drone_step * np.cos(self.theta_drone_step))
            y_target_drone = (self.r_drone_Step_previous * np.sin(self.theta_drone_step_previous)) 
                                    + (self.r_drone_step * np.sin(self.theta_drone_step))
            self.target_pos_drone = (x_target_drone, y_target_drone)
        
            ## publishing target position
            publish_msg = Point()
            publish_msg.x = x_target_drone
            publish_msg.y = y_target_drone
            publish_msg.z = 1.0

            self.pub.publish(publish_msg)

            # Storing positions
            X_array.append(x_target_drone) 
            Y_array.append(y_target_drone)

            self.pub_step = self.pub_step + 1
    def Check_Convergence(current_drone_temp):
        if np.sum(np.array(self.temp_grad_array_values) < 0.0) == 0:
            if temperature(self.r_drone_step_previous, self.theta_drone_step_previous) >= Source_Temperature_Threshold:
                print(f"Fire Source Detected : {self.r_drone_step_previous * np.cos(self.theta_drone_step_previous)}, {self.r_drone_step_previous * np.sin(self.theta_drone_step_previous)}")
                raise KeyboardInterrupt
        
        if self.pub_step == 13:
            if temperature(self.r_drone_step_previous, self.theta_drone_step_previous) > current_drone_temp:
                self.convergence_count = self.convergence_count + 1
                self.temp_value_previous = current_drone_temp
                
        if self.pub_step >= 14:
            if self.temp_value_previous > current_drone_temp:
                self.convergence_count = self.convergence_count + 1
            else:
                self.convergence_count = 0
            self.temp_value_previous = current_drone_temp
        
        if self.convergence_count >= 4:
            if self.temp_value_array[self.pub_step - 16] >= Source_Temperature_Threshold
                fire_temperature_loc = self.temp_r_theta_array[self.pub_step - 16]
                print(f"Fire Source Detected: {fire_temperature_loc}")
                raise KeyboardInterrupt
            


def main(args = None):
    rclpy.init(args = args)
    node = TargetPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Stopping node...")
    finally:
        temperature_distribution = np.zeros((1001, 1001))
        for i in range(0, temperature_distribution.shape[0]):
            for j in range(0, temperature_distribution.shape[1]):
                temperature_distribution[i, j] = temperature_distribution_plot(((-5) + (j/100)), ((-5) + (i/100)))
        sns.heatmap(temperature_distribution, cmap = "hot")
        plt.scatter(850, 800, color = "green")
        if len(X_array) > 0:
            ## Saving array for metrics.
            np.savez_compressed("Data_Epsilon_Greedy.npz", x_data = np.array(X_array), y_data = np.array(Y_array))
            plt.plot((500 + (np.array(X_array) * 100)), (500 + (np.array(Y_array) * 100)), 'b.-')
            plt.xlabel("X --->")
            plt.ylabel("Y --->")
            plt.title("Drone path during Fire Localization Epsilon Greedy - Algorithm 2")
            plt.grid(True)
            plt.savefig("/home/soham-banerjee/Algorithm/Algorithm_2/Trajectory_Epsilon_Greedy.png")
            plt.show()
        else:
            print("No data collected to plot.")
  
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
