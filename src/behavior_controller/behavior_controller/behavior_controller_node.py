#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Joy
import threading
import time
import math

try:
    import pygame
    import face_recognition
    import cv2
    import numpy as np
    import pickle
    HAS_PYGAME = True
    HAS_FACE_REC = True
except ImportError as e:
    HAS_PYGAME = False
    HAS_FACE_REC = False
    print(f"[WARNING] Optional dependencies not available: {e}")

class BehaviorControllerNode(Node):
    def __init__(self):
        super().__init__('behavior_controller_node')
        
        # Movement Constants
        self.Z_DEFAULT = -50.0
        self.Z_UP = -30.0
        self.Z_BOOT = -28.0
        self.X_DEFAULT = 62.0
        self.X_OFFSET = 0.0
        self.Y_START = 0.0
        self.Y_STEP = 40.0
        self.KEEP = 255
        
        # Turn Constants
        LENGTH_SIDE = 71.0
        temp_a = math.sqrt(pow(2 * self.X_DEFAULT + LENGTH_SIDE, 2) + pow(self.Y_STEP, 2))
        temp_b = 2 * (self.Y_START + self.Y_STEP) + LENGTH_SIDE
        temp_c = math.sqrt(
            pow(2 * self.X_DEFAULT + LENGTH_SIDE, 2) + 
            pow(2 * self.Y_START + self.Y_STEP + LENGTH_SIDE, 2)
        )
        temp_alpha = math.acos(
            (pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b
        )
        
        self.TURN_X1 = (temp_a - LENGTH_SIDE) / 2
        self.TURN_Y1 = self.Y_START + self.Y_STEP / 2
        self.TURN_X0 = self.TURN_X1 - temp_b * math.cos(temp_alpha)
        self.TURN_Y0 = temp_b * math.sin(temp_alpha) - self.TURN_Y1 - LENGTH_SIDE
        
        # Speed settings
        self.move_speed = 1.0
        self.speed_multiple = 1.0
        self.spot_turn_speed = 4.0
        self.leg_move_speed = 8.0
        self.body_move_speed = 3.0
        self.stand_seat_speed = 1.0
        
        # State variables
        self.site_now = [[0.0, 0.0, 0.0] for _ in range(4)]
        self.gesture_in_progress = False
        self.last_greeted_person = None
        self.greeting_cooldown = 10
        self.last_greeting_time = 0
        
        # Publishers
        self.leg_position_pub = self.create_publisher(
            Float64MultiArray,
            'leg_positions',
            10
        )
        
        self.behavior_status_pub = self.create_publisher(
            String,
            'behavior_status',
            10
        )
        
        # Subscribers
        self.leg_feedback_sub = self.create_subscription(
            Float64MultiArray,
            'leg_feedback',
            self.leg_feedback_callback,
            10
        )
        
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Xbox Controller setup
        self.setup_controller()
        
        # Face recognition setup
        self.setup_face_recognition()
        
        # Robot initialization
        self.initialize_robot()
        
        self.get_logger().info("Behavior controller node started")
    
    def setup_controller(self):
        """Initialize Xbox controller."""
        if HAS_PYGAME:
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.get_logger().info(f"Controller detected: {self.joystick.get_name()}")
                
                # Start controller thread
                self.controller_thread = threading.Thread(target=self.xbox_control_loop, daemon=True)
                self.controller_thread.start()
            else:
                self.joystick = None
                self.get_logger().warning("No joystick detected")
        else:
            self.joystick = None
    
    def setup_face_recognition(self):
        """Initialize face recognition."""
        if HAS_FACE_REC:
            try:
                with open("../../encodings.pickle", "rb") as f:
                    data = pickle.loads(f.read())
                self.known_face_encodings = data["encodings"]
                self.known_face_names = data["names"]
                
                # Initialize webcam
                self.cap = cv2.VideoCapture(0)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                
                # Start face recognition thread
                self.face_thread = threading.Thread(target=self.face_recognition_loop, daemon=True)
                self.face_thread.start()
                
                self.get_logger().info("Face recognition initialized")
            except Exception as e:
                self.get_logger().error(f"Face recognition setup failed: {e}")
                self.cap = None
        else:
            self.cap = None
    
    def initialize_robot(self):
        """Initialize robot to starting position."""
        self.set_site(0, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_BOOT)
        self.set_site(1, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_BOOT)
        self.set_site(2, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_BOOT)
        self.set_site(3, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_BOOT)
    
    def set_site(self, leg, x, y, z):
        """Send leg position command."""
        msg = Float64MultiArray()
        msg.data = [float(leg), float(x), float(y), float(z)]
        self.leg_position_pub.publish(msg)
    
    def wait_all_reach(self):
        """Simple wait - in real implementation would check feedback."""
        time.sleep(0.5)
    
    def leg_feedback_callback(self, msg):
        """Update current leg positions."""
        if len(msg.data) >= 12:
            for i in range(4):
                for j in range(3):
                    idx = i * 3 + j
                    self.site_now[i][j] = msg.data[idx]
    
    def joy_callback(self, msg):
        """Handle joystick input."""
        if len(msg.buttons) >= 4 and len(msg.axes) >= 5:
            # Buttons
            if msg.buttons[0]:  # A
                self.hand_wave(1)
            if msg.buttons[1]:  # B
                self.hand_shake(1)
            if msg.buttons[2]:  # X
                self.sit()
            if msg.buttons[3]:  # Y
                self.stand()
            
            # Axes (with deadzone)
            deadzone = 0.2
            lx, ly = msg.axes[0], msg.axes[1]
            rx = msg.axes[2] if len(msg.axes) > 2 else 0.0
            
            if ly < -deadzone:
                self.step_forward(1)
            elif ly > deadzone:
                self.step_back(1)
            
            if lx < -deadzone:
                self.body_left(5)
            elif lx > deadzone:
                self.body_right(5)
            
            if rx < -deadzone:
                self.turn_left(1)
            elif rx > deadzone:
                self.turn_right(1)
    
    def xbox_control_loop(self):
        """Xbox controller polling loop."""
        if not self.joystick:
            return
        
        deadzone = 0.2
        
        while rclpy.ok():
            try:
                pygame.event.pump()
                
                # Left stick Y: forward/back
                ly = self.joystick.get_axis(1)
                if ly < -deadzone:
                    self.step_forward(1)
                elif ly > deadzone:
                    self.step_back(1)
                
                # Left stick X: lateral movement
                lx = self.joystick.get_axis(0)
                if lx < -deadzone:
                    self.body_left(5)
                elif lx > deadzone:
                    self.body_right(5)
                
                # Right stick X: turning
                rx = self.joystick.get_axis(3)
                if rx < -deadzone:
                    self.turn_left(1)
                elif rx > deadzone:
                    self.turn_right(1)
                
                # Buttons
                if self.joystick.get_button(0):  # A
                    self.hand_wave(1)
                if self.joystick.get_button(1):  # B
                    self.hand_shake(1)
                if self.joystick.get_button(2):  # X
                    self.sit()
                if self.joystick.get_button(3):  # Y
                    self.stand()
                
                time.sleep(0.05)
            except Exception as e:
                self.get_logger().error(f"Controller error: {e}")
                break
    
    def face_recognition_loop(self):
        """Face recognition loop."""
        if not self.cap:
            return
        
        cv_scaler = 4
        
        while rclpy.ok():
            try:
                ret, frame = self.cap.read()
                if not ret:
                    continue
                
                # Process frame
                resized_frame = cv2.resize(frame, (0, 0), fx=(1/cv_scaler), fy=(1/cv_scaler))
                rgb_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
                
                face_locations = face_recognition.face_locations(rgb_frame)
                face_encodings = face_recognition.face_encodings(rgb_frame, face_locations, model="large")
                
                face_names = []
                for face_encoding in face_encodings:
                    matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
                    name = "Unknown"
                    
                    face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
                    best_match_index = np.argmin(face_distances)
                    if matches[best_match_index]:
                        name = self.known_face_names[best_match_index]
                    face_names.append(name)
                
                # Check for greetings
                self.check_and_greet(face_names)
                
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f"Face recognition error: {e}")
                break
    
    def check_and_greet(self, face_names):
        """Check if we should greet detected faces."""
        if self.gesture_in_progress:
            return
        
        allowed_names = ["Souri", "Vishnu", "Arjun"]
        known_faces = [name for name in face_names if name in allowed_names]
        
        if not known_faces:
            return
        
        person_to_greet = known_faces[0]
        current_time = time.time()
        
        if (
            self.last_greeted_person != person_to_greet
            or (current_time - self.last_greeting_time) >= self.greeting_cooldown
        ):
            gesture_thread = threading.Thread(target=self.perform_gesture, args=(person_to_greet,), daemon=True)
            gesture_thread.start()
    
    def perform_gesture(self, name):
        """Execute greeting gesture."""
        if self.gesture_in_progress:
            return
        
        self.gesture_in_progress = True
        
        try:
            self.get_logger().info(f"Greeting {name}")
            
            if name in ["Souri", "Vishnu", "Arjun"]:
                self.hand_wave(3)
                self.hand_shake(3)
                self.step_forward(6)
                self.step_back(6)
            
            self.last_greeted_person = name
            self.last_greeting_time = time.time()
            
        except Exception as e:
            self.get_logger().error(f"Gesture failed: {e}")
        finally:
            self.gesture_in_progress = False
    
    # High Level Movement Functions
    def sit(self):
        """Make robot sit."""
        self.move_speed = self.stand_seat_speed
        for leg in range(4):
            self.set_site(leg, self.KEEP, self.KEEP, self.Z_BOOT)
        self.wait_all_reach()
        
        status_msg = String()
        status_msg.data = "sitting"
        self.behavior_status_pub.publish(status_msg)
    
    def stand(self):
        """Make robot stand."""
        self.move_speed = self.stand_seat_speed
        for leg in range(4):
            self.set_site(leg, self.KEEP, self.KEEP, self.Z_DEFAULT)
        self.wait_all_reach()
        
        status_msg = String()
        status_msg.data = "standing"
        self.behavior_status_pub.publish(status_msg)
    
    def hand_wave(self, count):
        """Wave hand gesture."""
        if self.site_now[3][1] == self.Y_START:
            self.body_right(15)
            x_tmp = self.site_now[2][0]
            y_tmp = self.site_now[2][1]
            z_tmp = self.site_now[2][2]
            
            for _ in range(count):
                self.set_site(2, self.TURN_X1, self.TURN_Y1, 50)
                self.wait_all_reach()
                self.set_site(2, self.TURN_X0, self.TURN_Y0, 50)
                self.wait_all_reach()
            
            self.set_site(2, x_tmp, y_tmp, z_tmp)
            self.wait_all_reach()
            self.body_left(15)
        else:
            self.body_left(15)
            x_tmp = self.site_now[0][0]
            y_tmp = self.site_now[0][1] 
            z_tmp = self.site_now[0][2]
            
            for _ in range(count):
                self.set_site(0, self.TURN_X1, self.TURN_Y1, 50)
                self.wait_all_reach()
                self.set_site(0, self.TURN_X0, self.TURN_Y0, 50)
                self.wait_all_reach()
            
            self.set_site(0, x_tmp, y_tmp, z_tmp)
            self.wait_all_reach()
            self.body_right(15)
    
    def hand_shake(self, count):
        """Hand shake gesture."""
        if self.site_now[3][1] == self.Y_START:
            self.body_right(15)
            x_tmp = self.site_now[2][0]
            y_tmp = self.site_now[2][1]
            z_tmp = self.site_now[2][2]
            
            for _ in range(count):
                self.set_site(2, self.X_DEFAULT - 30, self.Y_START + 2 * self.Y_STEP, 55)
                self.wait_all_reach()
                self.set_site(2, self.X_DEFAULT - 30, self.Y_START + 2 * self.Y_STEP, 10)
                self.wait_all_reach()
            
            self.set_site(2, x_tmp, y_tmp, z_tmp)
            self.wait_all_reach()
            self.body_left(15)
        else:
            self.body_left(15)
            x_tmp = self.site_now[0][0]
            y_tmp = self.site_now[0][1]
            z_tmp = self.site_now[0][2]
            
            for _ in range(count):
                self.set_site(0, self.X_DEFAULT - 30, self.Y_START + 2 * self.Y_STEP, 55)
                self.wait_all_reach()
                self.set_site(0, self.X_DEFAULT - 30, self.Y_START + 2 * self.Y_STEP, 10)
                self.wait_all_reach()
            
            self.set_site(0, x_tmp, y_tmp, z_tmp)
            self.wait_all_reach()
            self.body_right(15)
    
    def turn_left(self, step):
        """Turn robot left."""
        self.move_speed = self.spot_turn_speed
        for _ in range(step):
            if self.site_now[3][1] == self.Y_START:
                self.set_site(3, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_UP)
                self.wait_all_reach()
                
                self.set_site(0, self.TURN_X1 - self.X_OFFSET, self.TURN_Y1, self.Z_DEFAULT)
                self.set_site(1, self.TURN_X0 - self.X_OFFSET, self.TURN_Y0, self.Z_DEFAULT)
                self.set_site(2, self.TURN_X1 + self.X_OFFSET, self.TURN_Y1, self.Z_DEFAULT)
                self.set_site(3, self.TURN_X0 + self.X_OFFSET, self.TURN_Y0, self.Z_UP)
                self.wait_all_reach()
                
                self.set_site(3, self.TURN_X0 + self.X_OFFSET, self.TURN_Y0, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.set_site(0, self.TURN_X1 + self.X_OFFSET, self.TURN_Y1, self.Z_DEFAULT)
                self.set_site(1, self.TURN_X0 + self.X_OFFSET, self.TURN_Y0, self.Z_DEFAULT)
                self.set_site(2, self.TURN_X1 - self.X_OFFSET, self.TURN_Y1, self.Z_DEFAULT)
                self.set_site(3, self.TURN_X0 - self.X_OFFSET, self.TURN_Y0, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.set_site(1, self.TURN_X0 + self.X_OFFSET, self.TURN_Y0, self.Z_UP)
                self.wait_all_reach()
                
                self.set_site(0, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_DEFAULT)
                self.set_site(1, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_UP)
                self.set_site(2, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_DEFAULT)
                self.set_site(3, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.set_site(1, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_DEFAULT)
                self.wait_all_reach()
            else:
                # Alternative sequence for different starting position
                self.set_site(0, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_UP)
                self.wait_all_reach()
                # ... (similar pattern)
    
    def turn_right(self, step):
        """Turn robot right."""
        self.move_speed = self.spot_turn_speed
        for _ in range(step):
            if self.site_now[2][1] == self.Y_START:
                self.set_site(2, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_UP)
                self.wait_all_reach()
                
                self.set_site(0, self.TURN_X0 - self.X_OFFSET, self.TURN_Y0, self.Z_DEFAULT)
                self.set_site(1, self.TURN_X1 - self.X_OFFSET, self.TURN_Y1, self.Z_DEFAULT)
                self.set_site(2, self.TURN_X0 + self.X_OFFSET, self.TURN_Y0, self.Z_UP)
                self.set_site(3, self.TURN_X1 + self.X_OFFSET, self.TURN_Y1, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.set_site(2, self.TURN_X0 + self.X_OFFSET, self.TURN_Y0, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.set_site(0, self.TURN_X0 + self.X_OFFSET, self.TURN_Y0, self.Z_DEFAULT)
                self.set_site(1, self.TURN_X1 + self.X_OFFSET, self.TURN_Y1, self.Z_DEFAULT)
                self.set_site(2, self.TURN_X0 - self.X_OFFSET, self.TURN_Y0, self.Z_DEFAULT)
                self.set_site(3, self.TURN_X1 - self.X_OFFSET, self.TURN_Y1, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.set_site(0, self.TURN_X0 + self.X_OFFSET, self.TURN_Y0, self.Z_UP)
                self.wait_all_reach()
                
                self.set_site(0, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_UP)
                self.set_site(1, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_DEFAULT)
                self.set_site(2, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_DEFAULT)
                self.set_site(3, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.set_site(0, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_DEFAULT)
                self.wait_all_reach()
            else:
                self.set_site(1, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_UP)
                self.wait_all_reach()
                
                self.set_site(0, self.TURN_X1 + self.X_OFFSET, self.TURN_Y1, self.Z_DEFAULT)
                self.set_site(1, self.TURN_X0 + self.X_OFFSET, self.TURN_Y0, self.Z_UP)
                self.set_site(2, self.TURN_X1 - self.X_OFFSET, self.TURN_Y1, self.Z_DEFAULT)
                self.set_site(3, self.TURN_X0 - self.X_OFFSET, self.TURN_Y0, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.set_site(1, self.TURN_X0 + self.X_OFFSET, self.TURN_Y0, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.set_site(0, self.TURN_X1 - self.X_OFFSET, self.TURN_Y1, self.Z_DEFAULT)
                self.set_site(1, self.TURN_X0 - self.X_OFFSET, self.TURN_Y0, self.Z_DEFAULT)
                self.set_site(2, self.TURN_X1 + self.X_OFFSET, self.TURN_Y1, self.Z_DEFAULT)
                self.set_site(3, self.TURN_X0 + self.X_OFFSET, self.TURN_Y0, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.set_site(3, self.TURN_X0 + self.X_OFFSET, self.TURN_Y0, self.Z_UP)
                self.wait_all_reach()
                
                self.set_site(0, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_DEFAULT)
                self.set_site(1, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_DEFAULT)
                self.set_site(2, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_DEFAULT)
                self.set_site(3, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_UP)
                self.wait_all_reach()
                
                self.set_site(3, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_DEFAULT)
                self.wait_all_reach()
    
    def step_forward(self, step):
        """Step forward."""
        self.move_speed = self.leg_move_speed
        for _ in range(step):
            if self.site_now[2][1] == self.Y_START:
                self.set_site(2, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_UP)
                self.wait_all_reach()
                self.set_site(2, self.X_DEFAULT + self.X_OFFSET, self.Y_START + 2 * self.Y_STEP, self.Z_UP)
                self.wait_all_reach()
                self.set_site(2, self.X_DEFAULT + self.X_OFFSET, self.Y_START + 2 * self.Y_STEP, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.move_speed = self.body_move_speed
                
                self.set_site(0, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_DEFAULT)
                self.set_site(1, self.X_DEFAULT + self.X_OFFSET, self.Y_START + 2 * self.Y_STEP, self.Z_DEFAULT)
                self.set_site(2, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_DEFAULT)
                self.set_site(3, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.move_speed = self.leg_move_speed
                
                self.set_site(1, self.X_DEFAULT + self.X_OFFSET, self.Y_START + 2 * self.Y_STEP, self.Z_UP)
                self.wait_all_reach()
                self.set_site(1, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_UP)
                self.wait_all_reach()
                self.set_site(1, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_DEFAULT)
                self.wait_all_reach()
            else:
                self.set_site(0, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_UP)
                self.wait_all_reach()
                self.set_site(0, self.X_DEFAULT + self.X_OFFSET, self.Y_START + 2 * self.Y_STEP, self.Z_UP)
                self.wait_all_reach()
                self.set_site(0, self.X_DEFAULT + self.X_OFFSET, self.Y_START + 2 * self.Y_STEP, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.move_speed = self.body_move_speed
                
                self.set_site(0, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_DEFAULT)
                self.set_site(1, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_DEFAULT)
                self.set_site(2, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_DEFAULT)
                self.set_site(3, self.X_DEFAULT + self.X_OFFSET, self.Y_START + 2 * self.Y_STEP, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.move_speed = self.leg_move_speed
                
                self.set_site(3, self.X_DEFAULT + self.X_OFFSET, self.Y_START + 2 * self.Y_STEP, self.Z_UP)
                self.wait_all_reach()
                self.set_site(3, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_UP)
                self.wait_all_reach()
                self.set_site(3, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_DEFAULT)
                self.wait_all_reach()
    
    def step_back(self, step):
        """Step backward."""
        self.move_speed = self.leg_move_speed
        for _ in range(step):
            if self.site_now[3][1] == self.Y_START:
                self.set_site(3, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_UP)
                self.wait_all_reach()
                self.set_site(3, self.X_DEFAULT + self.X_OFFSET, self.Y_START + 2 * self.Y_STEP, self.Z_UP)
                self.wait_all_reach()
                self.set_site(3, self.X_DEFAULT + self.X_OFFSET, self.Y_START + 2 * self.Y_STEP, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.move_speed = self.body_move_speed
                
                self.set_site(0, self.X_DEFAULT + self.X_OFFSET, self.Y_START + 2 * self.Y_STEP, self.Z_DEFAULT)
                self.set_site(1, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_DEFAULT)
                self.set_site(2, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_DEFAULT)
                self.set_site(3, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.move_speed = self.leg_move_speed
                
                self.set_site(0, self.X_DEFAULT + self.X_OFFSET, self.Y_START + 2 * self.Y_STEP, self.Z_UP)
                self.wait_all_reach()
                self.set_site(0, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_UP)
                self.wait_all_reach()
                self.set_site(0, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_DEFAULT)
                self.wait_all_reach()
            else:
                self.set_site(1, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_UP)
                self.wait_all_reach()
                self.set_site(1, self.X_DEFAULT + self.X_OFFSET, self.Y_START + 2 * self.Y_STEP, self.Z_UP)
                self.wait_all_reach()
                self.set_site(1, self.X_DEFAULT + self.X_OFFSET, self.Y_START + 2 * self.Y_STEP, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.move_speed = self.body_move_speed
                
                self.set_site(0, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_DEFAULT)
                self.set_site(1, self.X_DEFAULT - self.X_OFFSET, self.Y_START + self.Y_STEP, self.Z_DEFAULT)
                self.set_site(2, self.X_DEFAULT + self.X_OFFSET, self.Y_START + 2 * self.Y_STEP, self.Z_DEFAULT)
                self.set_site(3, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_DEFAULT)
                self.wait_all_reach()
                
                self.move_speed = self.leg_move_speed
                
                self.set_site(2, self.X_DEFAULT + self.X_OFFSET, self.Y_START + 2 * self.Y_STEP, self.Z_UP)
                self.wait_all_reach()
                self.set_site(2, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_UP)
                self.wait_all_reach()
                self.set_site(2, self.X_DEFAULT + self.X_OFFSET, self.Y_START, self.Z_DEFAULT)
                self.wait_all_reach()
    
    def body_left(self, offset):
        """Lean body left."""
        self.set_site(0, self.site_now[0][0] + offset, self.KEEP, self.KEEP)
        self.set_site(1, self.site_now[1][0] + offset, self.KEEP, self.KEEP)
        self.set_site(2, self.site_now[2][0] - offset, self.KEEP, self.KEEP)
        self.set_site(3, self.site_now[3][0] - offset, self.KEEP, self.KEEP)
        self.wait_all_reach()
    
    def body_right(self, offset):
        """Lean body right."""
        self.set_site(0, self.site_now[0][0] - offset, self.KEEP, self.KEEP)
        self.set_site(1, self.site_now[1][0] - offset, self.KEEP, self.KEEP)
        self.set_site(2, self.site_now[2][0] + offset, self.KEEP, self.KEEP)
        self.set_site(3, self.site_now[3][0] + offset, self.KEEP, self.KEEP)
        self.wait_all_reach()

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
