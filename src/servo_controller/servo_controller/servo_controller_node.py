#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import math
import threading
import time

try:
    import Adafruit_PCA9685
    HAS_HARDWARE = True
except ImportError:
    HAS_HARDWARE = False
    print("[WARNING] Adafruit_PCA9685 not available. Running in simulation mode.")

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller_node')
        
        # Hardware initialization
        if HAS_HARDWARE:
            try:
                self.pwm = Adafruit_PCA9685.PCA9685(busnum=1)
                self.pwm.set_pwm_freq(50)
                self.get_logger().info("PCA9685 initialized successfully")
            except Exception as e:
                self.get_logger().error(f"Error initializing PCA9685: {e}")
                self.pwm = None
        else:
            self.pwm = None
            
        # Constants
        self.PI = math.pi
        self.LEG_CHANNELS = [
            [0, 1, 2],   # Leg 0
            [3, 4, 5],   # Leg 1
            [6, 7, 8],   # Leg 2
            [9, 10, 11], # Leg 3
        ]
        
        # Current servo angles (for simulation)
        self.servo_angles = [[90, 90, 90] for _ in range(4)]
        
        # Subscribers
        self.servo_sub = self.create_subscription(
            Float64MultiArray,
            'servo_angles',
            self.servo_callback,
            10
        )
        
        # Publishers
        self.servo_feedback_pub = self.create_publisher(
            Float64MultiArray,
            'servo_feedback',
            10
        )
        
        # Timer for feedback
        self.feedback_timer = self.create_timer(0.02, self.publish_feedback)
        
        self.get_logger().info("Servo controller node started")
    
    def angle_to_pulse(self, angle):
        """Converts degrees to PCA9685 pulse length (0-4096)."""
        pulse = int((angle * 2.5) + 150)
        return max(0, min(4096, pulse))
    
    def servo_callback(self, msg):
        """Process servo angle commands."""
        if len(msg.data) != 12:  # 4 legs * 3 servos
            self.get_logger().error("Invalid servo command size")
            return
            
        for leg in range(4):
            for servo in range(3):
                idx = leg * 3 + servo
                angle = msg.data[idx]
                
                # Update simulation state
                self.servo_angles[leg][servo] = angle
                
                # Send to hardware
                if self.pwm:
                    try:
                        self.pwm.set_pwm(
                            self.LEG_CHANNELS[leg][servo], 
                            0, 
                            self.angle_to_pulse(angle)
                        )
                    except Exception as e:
                        self.get_logger().error(f"Error setting servo: {e}")
    
    def publish_feedback(self):
        """Publish current servo positions."""
        msg = Float64MultiArray()
        data = []
        for leg in range(4):
            for servo in range(3):
                data.append(self.servo_angles[leg][servo])
        msg.data = data
        self.servo_feedback_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
