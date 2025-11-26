#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Float64MultiArray
import math
import threading
import time

class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        
        # Robot Dimensions (mm)
        self.LENGTH_A = 55.0
        self.LENGTH_B = 77.5
        self.LENGTH_C = 27.5
        self.LENGTH_SIDE = 71.0
        self.Z_ABSOLUTE = -28.0
        
        # Movement Constants
        self.Z_DEFAULT = -50.0
        self.Z_UP = -30.0
        self.Z_BOOT = self.Z_ABSOLUTE
        self.X_DEFAULT = 62.0
        self.X_OFFSET = 0.0
        self.Y_START = 0.0
        self.Y_STEP = 40.0
        
        # Servo Constants
        self.KEEP = 255
        self.PI = math.pi
        
        # Turn Constants calculation
        temp_a = math.sqrt(pow(2 * self.X_DEFAULT + self.LENGTH_SIDE, 2) + pow(self.Y_STEP, 2))
        temp_b = 2 * (self.Y_START + self.Y_STEP) + self.LENGTH_SIDE
        temp_c = math.sqrt(
            pow(2 * self.X_DEFAULT + self.LENGTH_SIDE, 2) + 
            pow(2 * self.Y_START + self.Y_STEP + self.LENGTH_SIDE, 2)
        )
        temp_alpha = math.acos(
            (pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b
        )
        
        self.TURN_X1 = (temp_a - self.LENGTH_SIDE) / 2
        self.TURN_Y1 = self.Y_START + self.Y_STEP / 2
        self.TURN_X0 = self.TURN_X1 - temp_b * math.cos(temp_alpha)
        self.TURN_Y0 = temp_b * math.sin(temp_alpha) - self.TURN_Y1 - self.LENGTH_SIDE
        
        # Global State Variables
        self.site_now = [[0.0, 0.0, 0.0] for _ in range(4)]
        self.site_expect = [[0.0, 0.0, 0.0] for _ in range(4)]
        self.temp_speed = [[0.0, 0.0, 0.0] for _ in range(4)]
        
        self.move_speed = 1.0
        self.speed_multiple = 1.0
        self.spot_turn_speed = 4.0
        self.leg_move_speed = 8.0
        self.body_move_speed = 3.0
        self.stand_seat_speed = 1.0
        
        # Subscribers
        self.leg_position_sub = self.create_subscription(
            Float64MultiArray,
            'leg_positions',
            self.leg_position_callback,
            10
        )
        
        # Publishers
        self.servo_pub = self.create_publisher(
            Float64MultiArray,
            'servo_angles',
            10
        )
        
        self.leg_feedback_pub = self.create_publisher(
            Float64MultiArray,
            'leg_feedback',
            10
        )
        
        # Initialize robot position
        self.robot_setup()
        
        # Start servo service loop
        self.servo_thread = threading.Thread(target=self.servo_service_loop, daemon=True)
        self.servo_thread.start()
        
        self.get_logger().info("Kinematics node started")
    
    def cartesian_to_polar(self, x, y, z):
        """Inverse Kinematics: Calculates alpha, beta, gamma angles."""
        w = (1 if x >= 0 else -1) * (math.sqrt(pow(x, 2) + pow(y, 2)))
        v = w - self.LENGTH_C
        
        try:
            alpha_rad = math.atan2(z, v) + math.acos(
                (pow(self.LENGTH_A, 2) - pow(self.LENGTH_B, 2) + pow(v, 2) + pow(z, 2))
                / 2
                / self.LENGTH_A
                / math.sqrt(pow(v, 2) + pow(z, 2))
            )
            beta_rad = math.acos(
                (pow(self.LENGTH_A, 2) + pow(self.LENGTH_B, 2) - pow(v, 2) - pow(z, 2))
                / 2
                / self.LENGTH_A
                / self.LENGTH_B
            )
        except ValueError:
            self.get_logger().error("Math Domain Error in IK - Target unreachable")
            return 90, 90, 90
        
        gamma_rad = math.atan2(y, x) if w >= 0 else math.atan2(-y, -x)
        
        alpha = alpha_rad / self.PI * 180
        beta = beta_rad / self.PI * 180
        gamma = gamma_rad / self.PI * 180
        
        return alpha, beta, gamma
    
    def polar_to_servo(self, leg, alpha, beta, gamma):
        """Maps kinematic angles to physical servo limits/orientations."""
        if leg == 0:
            alpha = 90 - alpha
            beta = beta
            gamma += 90
        elif leg == 1:
            alpha += 90
            beta = 180 - beta
            gamma = 90 - gamma
        elif leg == 2:
            alpha += 90
            beta = 180 - beta
            gamma = 90 - gamma
        elif leg == 3:
            alpha = 90 - alpha
            beta = beta
            gamma += 90
        
        return alpha, beta, gamma
    
    def servo_service_loop(self):
        """Background thread for servo interpolation at ~50Hz."""
        while rclpy.ok():
            servo_angles = []
            
            for i in range(4):
                for j in range(3):
                    if abs(self.site_now[i][j] - self.site_expect[i][j]) >= abs(self.temp_speed[i][j]):
                        self.site_now[i][j] += self.temp_speed[i][j]
                    else:
                        self.site_now[i][j] = self.site_expect[i][j]
                
                alpha, beta, gamma = self.cartesian_to_polar(
                    self.site_now[i][0], self.site_now[i][1], self.site_now[i][2]
                )
                alpha, beta, gamma = self.polar_to_servo(i, alpha, beta, gamma)
                servo_angles.extend([alpha, beta, gamma])
            
            # Publish servo angles
            msg = Float64MultiArray()
            msg.data = servo_angles
            self.servo_pub.publish(msg)
            
            # Publish leg feedback
            leg_msg = Float64MultiArray()
            leg_data = []
            for i in range(4):
                leg_data.extend(self.site_now[i])
            leg_msg.data = leg_data
            self.leg_feedback_pub.publish(leg_msg)
            
            time.sleep(0.02)
    
    def set_site(self, leg, x, y, z):
        """Sets the target position and calculates speed vector."""
        length_x = 0 if x == self.KEEP else x - self.site_now[leg][0]
        length_y = 0 if y == self.KEEP else y - self.site_now[leg][1]
        length_z = 0 if z == self.KEEP else z - self.site_now[leg][2]
        
        length = math.sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2))
        if length == 0:
            length = 0.001
        
        self.temp_speed[leg][0] = length_x / length * self.move_speed * self.speed_multiple
        self.temp_speed[leg][1] = length_y / length * self.move_speed * self.speed_multiple
        self.temp_speed[leg][2] = length_z / length * self.move_speed * self.speed_multiple
        
        if x != self.KEEP:
            self.site_expect[leg][0] = x
        if y != self.KEEP:
            self.site_expect[leg][1] = y
        if z != self.KEEP:
            self.site_expect[leg][2] = z
    
    def wait_reach(self, leg):
        """Blocking wait until specific leg reaches target."""
        while rclpy.ok():
            if (
                self.site_now[leg][0] == self.site_expect[leg][0]
                and self.site_now[leg][1] == self.site_expect[leg][1]
                and self.site_now[leg][2] == self.site_expect[leg][2]
            ):
                break
            time.sleep(0.001)
    
    def wait_all_reach(self):
        """Blocking wait until ALL legs reach target."""
        for i in range(4):
            self.wait_reach(i)
    
    def robot_setup(self):
        """Initialize robot to default position."""
        self.set_site(0, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_BOOT)
        self.set_site(1, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_BOOT)
        self.set_site(2, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_BOOT)
        self.set_site(3, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_BOOT)
        
        for i in range(4):
            for j in range(3):
                self.site_now[i][j] = self.site_expect[i][j]
    
    def leg_position_callback(self, msg):
        """Process leg position commands from behavior controller."""
        if len(msg.data) < 4:  # leg, x, y, z
            return
            
        leg = int(msg.data[0])
        x = msg.data[1] if msg.data[1] != self.KEEP else self.KEEP
        y = msg.data[2] if msg.data[2] != self.KEEP else self.KEEP
        z = msg.data[3] if msg.data[3] != self.KEEP else self.KEEP
        
        if leg >= 0 and leg < 4:
            self.set_site(leg, x, y, z)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
