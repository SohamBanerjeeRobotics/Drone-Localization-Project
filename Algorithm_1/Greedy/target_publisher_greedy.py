import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

#### Global Parameters for plotting ####
X_array = [] # Initializing X array
Y_array = [] # Initializing Y array
fire_temperature_loc_x = None # Detected fire source location.
fire_temperature_loc_y = None # Detected fire source location.
Global_Counter = 0
########################################
#### Global Variable for convergence check.
Source_Temperature_Threshold = 160.0 # user defined based on experiments.

#### Defining some global functions ######
def temperature_distribution_plot(x_pos_drone, y_pos_drone, T0 = 20.0, SX = 13.5, SY = 13.0, ST = 100.0, Z = 1.0, DTMax = 200.0, LT = 2.0, KH = 0.1):
    # Position of drone w.r.t fire source
    R_drone_fire = np.sqrt(((x_pos_drone - SX) ** 2) + ((y_pos_drone - SY) ** 2))
    A_param = (5.386 * (((Z / LT) + 0.558) ** 3.205)) * np.exp((-4.057) * (Z / LT))
    T_drone = T0 + (DTMax * (np.log(100) / np.log(1 + ST)) * np.exp((-KH) * R_drone_fire) * A_param)
    return T_drone
  
  
def temperature(r_drone, theta_drone, T0 = 20.0, SX = 13.5, SY = 13.0, ST = 100.0, Z = 1.0, DTMax = 200.0, LT = 2.0, KH = 0.1):
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
        self.r_drone_step = 0.3 # Initial step size.
        
        ### Flags for reinitializing random walk
        self.r_drone_step_previous = 0.0
        self.theta_drone_step_previous = 0.0
        
        ### variables to check convergence.
        self.temp_grad_array_values = []
        self.temp_r_theta_array = []
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
                print(f"Reached : {self.target_pos_drone}, Step : {self.pub_step}")
                temperature_drone_r_x = (self.r_drone_step_previous * np.cos(self.theta_drone_step_previous)) + (self.r_drone_step * np.cos(self.theta_drone_step))
                temperature_drone_r_y = (self.r_drone_step_previous * np.sin(self.theta_drone_step_previous)) + (self.r_drone_step * np.sin(self.theta_drone_step))
                temperature_drone_r = np.sqrt((temperature_drone_r_x ** 2) + (temperature_drone_r_y ** 2))
                temperature_drone_theta = (np.arctan2(temperature_drone_r_y, temperature_drone_r_x) + (2 * np.pi)) % (2 * np.pi)    
                temperature_drone = temperature(temperature_drone_r, temperature_drone_theta)
                if self.pub_step <= 72:
                    temperature_drone_previous = temperature(self.r_drone_step_previous, self.theta_drone_step_previous)
                    temperature_gradient_drone = temperature_gradient(temperature_drone, temperature_drone_previous, self.r_drone_step)
                    self.temp_grad_array_values.append(temperature_gradient_drone)
                    
                    self.temp_grad_array = self.temp_grad_array + (temperature_gradient_drone * np.cos(self.theta_drone_step - np.arange(0, 2 * np.pi, (5 * np.pi) / 180)))
                # Convergence purpose
                if self.pub_step >= 72:
                    if self.pub_step == 72:
                        print(f"moving in circle completed. all gradients info have been successfully collected.")
                        self.temp_r_theta_array.append((self.r_drone_step_previous, self.theta_drone_step_previous))
                        self.temp_value_array.append(temperature(self.r_drone_step_previous, self.theta_drone_step_previous))
                    else:
                        print(f"moving in a straight line in the max grad direction.")
                        self.temp_r_theta_array.append((temperature_drone_r, temperature_drone_theta))
                        self.temp_value_array.append(temperature_drone)
                    self.Check_Convergence(temperature_drone)
                
                # Code to reinitialize random walk
                if self.pub_step == 77:
                    r_drone_step_previous_x = (self.r_drone_step_previous * np.cos(self.theta_drone_step_previous)) + (self.r_drone_step * np.cos(self.theta_drone_step))
                    r_drone_step_previous_y = (self.r_drone_step_previous * np.sin(self.theta_drone_step_previous)) + (self.r_drone_step * np.sin(self.theta_drone_step))
                    self.r_drone_step_previous = np.sqrt((r_drone_step_previous_x ** 2) + (r_drone_step_previous_y ** 2))
                    self.theta_drone_step_previous = (np.arctan2(r_drone_step_previous_y, r_drone_step_previous_x) + (2 * np.pi)) % (2 * np.pi)
                    # It makes the random walk to happen.
                    self.pub_step = 0 
                    self.r_drone_step = 0.3
                    self.theta_drone_step = None
                    self.temp_grad_array = np.zeros(72)
                    self.temp_grad_array_values = []
                    # Convergence purpose
                    self.convergence_count = 0
                    self.temp_r_theta_array = []
                    self.temp_value_array = []
                    return # break out of the function.
                
                if self.pub_step  >= 72: # Index after which circle is completed.
                    self.r_drone_step = self.r_drone_step + 0.6 # step increment between two random walks.
                    self.theta_drone_step = np.argmax(self.temp_grad_array) * ((5 * np.pi) / 180)
          
                    # Generalized drone target position
                    x_target_drone = (self.r_drone_step_previous * np.cos(self.theta_drone_step_previous)) + (self.r_drone_step * np.cos(self.theta_drone_step))
                    y_target_drone = (self.r_drone_step_previous * np.sin(self.theta_drone_step_previous)) + (self.r_drone_step * np.sin(self.theta_drone_step))
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
                    self.theta_drone_step = theta_drone_array[self.pub_step]

                    # Generalized drone target position
                    x_target_drone = (self.r_drone_step_previous * np.cos(self.theta_drone_step_previous)) + (self.r_drone_step * np.cos(self.theta_drone_step))
                    y_target_drone = (self.r_drone_step_previous * np.sin(self.theta_drone_step_previous)) + (self.r_drone_step * np.sin(self.theta_drone_step))
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
            self.theta_drone_step = theta_drone_array[self.pub_step]

            # Generalized drone target position
            x_target_drone = (self.r_drone_step_previous * np.cos(self.theta_drone_step_previous)) + (self.r_drone_step * np.cos(self.theta_drone_step))
            y_target_drone = (self.r_drone_step_previous * np.sin(self.theta_drone_step_previous)) + (self.r_drone_step * np.sin(self.theta_drone_step))
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
    def Check_Convergence(self, current_drone_temp):
        global Global_Counter, fire_temperature_loc_x, fire_temperature_loc_y # Declaring Global Variables.
        Global_Counter = Global_Counter + 1
        if np.sum(np.array(self.temp_grad_array_values) < 0.0) == 0:
            if temperature(self.r_drone_step_previous, self.theta_drone_step_previous) >= Source_Temperature_Threshold:
                fire_temperature_loc_x = self.r_drone_step_previous * np.cos(self.theta_drone_step_previous)
                fire_temperature_loc_y = self.r_drone_step_previous * np.sin(self.theta_drone_step_previous)
                print(f"Fire Source Detected : ({fire_temperature_loc_x}, {fire_temperature_loc_y})")
                raise KeyboardInterrupt
        
        if self.pub_step == 73:
            if temperature(self.r_drone_step_previous, self.theta_drone_step_previous) > current_drone_temp:
                self.convergence_count = self.convergence_count + 1
            self.temp_value_previous = current_drone_temp
                
        if self.pub_step >= 74:
            if self.temp_value_previous > current_drone_temp:
                self.convergence_count = self.convergence_count + 1
            else:
                self.convergence_count = 0
            self.temp_value_previous = current_drone_temp
        
        if self.convergence_count >= 4:
            if self.temp_value_array[self.pub_step - 76] >= Source_Temperature_Threshold:
                fire_temperature_loc = self.temp_r_theta_array[self.pub_step - 76]
                fire_temperature_loc_x = fire_temperature_loc[0] * np.cos(fire_temperature_loc[1])
                fire_temperature_loc_y = fire_temperature_loc[0] * np.sin(fire_temperature_loc[1])
                print(f"Fire Source Detected: ({fire_temperature_loc_x}, {fire_temperature_loc_y})")
                raise KeyboardInterrupt
        
        if Global_Counter >= 100:
            fire_temperature_loc = self.temp_r_theta_array[self.pub_step - 72]
            fire_temperature_loc_x = fire_temperature_loc[0] * np.cos(fire_temperature_loc[1])
            fire_temperature_loc_y = fire_temperature_loc[0] * np.sin(fire_temperature_loc[1])
            print(f"Fire source could not be detected. Terminating the process.")
            raise KeyboardInterrupt

def main(args = None):
    rclpy.init(args = args)
    node = TargetPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Stopping node...")
    finally:
        temperature_distribution = np.zeros((5001, 5001))
        for i in range(0, temperature_distribution.shape[0]):
            for j in range(0, temperature_distribution.shape[1]):
                temperature_distribution[i, j] = temperature_distribution_plot(((-25) + (j/100)), ((-25) + (i/100)))
        sns.heatmap(temperature_distribution, cmap = "hot", xticklabels = False, yticklabels = False)
        plt.scatter(3850, 3800, color = "green")
        if len(X_array) > 0:
            ## Saving array for metrics.
            np.savez_compressed("/home/soham-banerjee/Algorithm/Algorithm_1/Greedy/Case_1/Data_Case1.npz", x_data = np.array(X_array), y_data = np.array(Y_array), x_actual_source = 13.5, y_actual_source = 13.0, x_detected_source = fire_temperature_loc_x, y_detected_source = fire_temperature_loc_y)
            plt.plot((2500 + (np.array(X_array) * 100)), (2500 + (np.array(Y_array) * 100)), 'b.-')
            
            ## To add custom tick labels.
            tick_positions = [0, 1250, 2500, 3750, 5000]  # Pixel positions
            tick_labels = ['-25', '-12.5', '0', '12.5', '25']  # Real values
    
            plt.xticks(tick_positions, tick_labels)
            plt.yticks(tick_positions, tick_labels)

            plt.xlabel("X --->")
            plt.ylabel("Y --->")
            plt.title("Drone path during Fire Localization Greedy - Algorithm-1 (Case 1)")
            plt.grid(True)
            plt.savefig("/home/soham-banerjee/Algorithm/Algorithm_1/Greedy/Case_1/Trajectory_Case1.png")
            plt.show()
        else:
            print("No data collected to plot.")
  
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
