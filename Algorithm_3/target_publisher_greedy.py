import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

#### Global Parameters for plotting ####
X_array = [] # Initializing X array
Y_array = [] # Initializing Y array
Source_Temperature_Threshold = 200.0
########################################

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
        self.r_drone_step = 0.1 # step size at each run.

        ### variables updated at each publication ###
        self.temp_grad_array = np.zeros(72) # creates an array to store grad data
        self.pub_step = 0 # initializing publishing step
        
        ## Initializing variables
        self.temp_previous_drone = temperature(0.0, 0.0)
        self.dr = 0.1
        
        ## Convergence purpose
        self.temp_array = []
        self.convergence_count = 0
        
    def gps_callback(self, msg: PointStamped):
        self.current_gps_drone = (msg.point.x, msg.point.y) # storing (x,y) coordinates.
        self.greedy() # Calls the greedy function.


    def greedy(self):
        if self.pub_step != 0:
            distance_remaining = np.linalg.norm(np.array(self.target_pos_drone) - np.array(self.current_gps_drone))
            # To check whether the drone has reached target position or not
            if distance_remaining <= 0.02:
                print(f"Reached : {self.target_pos_drone}")
                temperature_drone = temperature(self.r_drone_step, self.theta_drone_step)
                temperature_gradient_drone = temperature_gradient(temperature_drone, self.temp_previous_drone, self.dr)
                self.temp_grad_array = self.temp_grad_array + (temperature_gradient_drone * np.cos(self.theta_drone_step - np.arange(0, 2 * np.pi, (5 * np.pi) / 180)))
                if self.pub_step >= 12:
                    if self.pub_step == 12:
                        self.temp_array.append(self.temp_previous_drone)
                    else:
                        self.temp_array.append(temperature_drone)
                if self.pub_step  >= 12:
                    self.Check_Convergence(temperature_drone)
                    self.r_drone_step = self.r_drone_step + 0.25
                    self.theta_drone_step = np.argmax(self.temp_grad_array) * ((5 * np.pi) / 180)
                    
                    # drone target position
                    x_target_drone = self.r_drone_step * np.cos(self.theta_drone_step)
                    y_target_drone = self.r_drone_step * np.sin(self.theta_drone_step)
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
                    
                    if self.pub_step >= 13:
                        self.temp_previous_drone = temperature_drone
                    self.pub_step = self.pub_step + 1
                    self.dr = 0.25
                    
                else:
                    # update r_drone_step value
                    self.r_drone_step = self.r_drone_step + 0.0
                    # choose a random theta
                    theta_drone_array = np.arange(0, 2 * np.pi, (5 * np.pi) / 180)
                    self.theta_drone_step = np.random.choice(theta_drone_array) 

                    # drone target position
                    x_target_drone = self.r_drone_step * np.cos(self.theta_drone_step)
                    y_target_drone = self.r_drone_step * np.sin(self.theta_drone_step)
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

            # drone target position
            x_target_drone = self.r_drone_step * np.cos(self.theta_drone_step)
            y_target_drone = self.r_drone_step * np.sin(self.theta_drone_step)
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
        if self.pub_step == 13:
            if self.temp_previous_drone > current_drone_temp:
                self.convergence_count = self.convergence_count + 1
                
        if self.pub_step >= 14:
            if self.temp_previous_drone > current_drone_temp:
                self.convergence_count = self.convergence_count + 1
            else:
                self.convergence_count = 0
        
        if self.convergence_count >= 4:
            if self.temp_array[self.pub_step - 16] >= Source_Temperature_Threshold
                fire_temperature_loc = (self.r_drone_step, self.theta_drone_step)
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
            np.savez_compressed("Data_Greedy.npz", x_data = np.array(X_array), y_data = np.array(Y_array))
            plt.plot((500 + (np.array(X_array) * 100)), (500 + (np.array(Y_array) * 100)), 'b.-')
            plt.xlabel("X --->")
            plt.ylabel("Y --->")
            plt.title("Drone path during Fire Localization Greedy - Algorithm 3")
            plt.grid(True)
            plt.savefig("/home/soham-banerjee/Algorithm/Algorithm_3/Trajectory_Greedy.png")
            plt.show()
        else:
            print("No data collected to plot.")
  
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
