import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist, Point

X_Control_Energy_Array = []
Y_Control_Energy_Array = []
Z_Control_Energy_Array = []


class CrazyfliePositionController(Node):
    def __init__(self):
        super().__init__('crazyflie_position_controller')

        # Proportional gain parameter
        self.declare_parameter('Kp', 0.1)
        self.Kp = self.get_parameter('Kp').value

        # Derivative controller
        self.declare_parameter('Kd', 0.0)
        self.Kd = self.get_parameter('Kd').value

        # Subscribers
        self.target_sub = self.create_subscription(
            Point, '/target_pos', self.target_callback, 10)
        self.gps_sub = self.create_subscription(
            PointStamped, '/Crazyflie/gps', self.gps_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Storage for latest values
        self.target_pos = None
        self.current_pos = None
        self.last_error_x = None
        self.last_error_y = None
        self.last_error_z = None

    def target_callback(self, msg: Point):
        self.target_pos = msg
        self.compute_and_publish()

    def gps_callback(self, msg: PointStamped):
        self.current_pos = msg.point
        self.compute_and_publish()

    def compute_and_publish(self):
        if self.target_pos is None or self.current_pos is None:
            return

        # Error = target - current
        error_x = self.target_pos.x - self.current_pos.x
        error_y = self.target_pos.y - self.current_pos.y
        error_z = self.target_pos.z - self.current_pos.z

        # Error_dot = (current - previous) / time
        dt = (1 / 30)  # Assuming 0.1 s between updates

        if self.last_error_x is None:
            error_x_dot = 0.0
        else:
            error_x_dot = (error_x - self.last_error_x) / dt

        if self.last_error_y is None:
            error_y_dot = 0.0
        else:
            error_y_dot = (error_y - self.last_error_y) / dt

        if self.last_error_z is None:
            error_z_dot = 0.0
        else:
            error_z_dot = (error_z - self.last_error_z) / dt

        # Simple proportional-derivative control law
        cmd = Twist()
        cmd.linear.x = self.Kp * error_x + self.Kd * error_x_dot
        # velocity capping - x
        if cmd.linear.x > 0.2:
            cmd.linear.x = 0.2
        elif cmd.linear.x < -0.2:
            cmd.linear.x = -0.2
            
        cmd.linear.y = self.Kp * error_y + self.Kd * error_y_dot
        # velocity capping - y
        if cmd.linear.y > 0.2:
            cmd.linear.y = 0.2
        elif cmd.linear.y < -0.2:
            cmd.linear.y = -0.2
            
        cmd.linear.z = self.Kp * error_z + self.Kd * error_z_dot
        # velocity capping - z
        if cmd.linear.z > 0.2:
            cmd.linear.z = 0.2
        elif cmd.linear.z < -0.2:
            cmd.linear.z = -0.2   

        # Updating last errors
        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_error_z = error_z

        self.cmd_vel_pub.publish(cmd)
        
        X_Control_Energy_Array.append(cmd.linear.x)
        Y_Control_Energy_Array.append(cmd.linear.y)
        Z_Control_Energy_Array.append(cmd.linear.z)
        
        self.get_logger().info(
            f"Published cmd_vel: x={cmd.linear.x:.2f}, y={cmd.linear.y:.2f}, z={cmd.linear.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CrazyfliePositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Stopping node...")
        np.savez_compressed("/home/soham-banerjee/Data_Case_4e_Input.npz", x_control_data = np.array(X_Control_Energy_Array), y_control_data = np.array(X_Control_Energy_Array), z_control_data = np.array(Z_Control_Energy_Array))
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
