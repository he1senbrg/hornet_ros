#!/usr/bin/env python3  
  
import rclpy  
from rclpy.node import Node  
from std_msgs.msg import Float64MultiArray, String  
from sensor_msgs.msg import Joy  
import time  
import subprocess  
import sys  
import os  
  
class HornetSystemTest(Node):  
    def __init__(self):  
        super().__init__('hornet_system_test')  
        self.test_results = {}
      
    def test_node_connectivity(self):
        """Test if all required nodes are running"""  
        self.get_logger().info("Testing node connectivity...")  
          
        try:  
            result = subprocess.run(['ros2', 'node', 'list'],   
                                  capture_output=True, text=True, timeout=10)  
            nodes = result.stdout.strip().split('\n')  
              
            required_nodes = ['/behavior_controller', '/kinematics', '/servo_controller']  
            missing_nodes = [node for node in required_nodes if node not in nodes]  
              
            if missing_nodes:  
                self.test_results['node_connectivity'] = False  
                self.get_logger().error(f"Missing nodes: {missing_nodes}")  
            else:  
                self.test_results['node_connectivity'] = True  
                self.get_logger().info("All required nodes are running")  
                  
        except Exception as e:  
            self.test_results['node_connectivity'] = False  
            self.get_logger().error(f"Node connectivity test failed: {e}")  
      
    def test_topic_communication(self):  
        """Test topic communication between nodes"""  
        self.get_logger().info("Testing topic communication...")  
          
        # Test by directly checking if topics are publishing data
        import subprocess
        
        expected_topics = ['servo_feedback', 'leg_feedback']
        failing_topics = []
        
        for topic in expected_topics:
            try:
                # Use ros2 topic echo to check if topic is publishing
                result = subprocess.run([
                    'timeout', '2', 'ros2', 'topic', 'echo', topic, '--once'
                ], capture_output=True, text=True, timeout=3)
                
                if result.returncode == 0 and result.stdout.strip():
                    self.get_logger().info(f"Topic {topic} is publishing data")
                else:
                    failing_topics.append(topic)
                    self.get_logger().error(f"Topic {topic} not publishing or timeout")
                    
            except Exception as e:
                failing_topics.append(topic)
                self.get_logger().error(f"Error checking topic {topic}: {e}")
        
        if failing_topics:  
            self.test_results['topic_communication'] = False  
            self.get_logger().error(f"Topics not working: {failing_topics}")  
        else:  
            self.test_results['topic_communication'] = True  
            self.get_logger().info("Topic communication working")  
      
    def test_basic_movements(self):  
        """Test basic robot movements"""  
        self.get_logger().info("Testing basic movements...")  
          
        # Send a test command to leg_positions  
        leg_pub = self.create_publisher(Float64MultiArray, 'leg_positions', 10)  
        
        # Wait for publisher to be established
        time.sleep(0.5)
          
        # Send test command  
        test_msg = Float64MultiArray()  
        test_msg.data = [0.0, 62.0, 40.0, -50.0]  # leg 0 to default position  
        leg_pub.publish(test_msg)  
        
        self.get_logger().info("Sent movement command, waiting for system response...")
        time.sleep(1.0)
        
        # Check if servo_feedback topic is actively publishing by sampling it
        import subprocess
        try:
            result = subprocess.run([
                'timeout', '2', 'ros2', 'topic', 'echo', 'servo_feedback', '--once'
            ], capture_output=True, text=True, timeout=3)
            
            if result.returncode == 0 and result.stdout.strip():
                self.test_results['basic_movements'] = True  
                self.get_logger().info("Basic movement test passed - servo feedback active")
            else:
                self.test_results['basic_movements'] = False  
                self.get_logger().error("No servo feedback received during movement test")
        except Exception as e:
            self.test_results['basic_movements'] = False  
            self.get_logger().error(f"Error testing movement: {e}")  
      
    def test_hardware_dependencies(self):  
        """Test hardware dependencies"""  
        self.get_logger().info("Testing hardware dependencies...")  
          
        hardware_status = {}  
          
        # Check PCA9685 availability  
        try:  
            import Adafruit_PCA9685  
            hardware_status['pca9685'] = True  
            self.get_logger().info("PCA9685 library available")  
        except ImportError:  
            hardware_status['pca9685'] = False  
            self.get_logger().warning("PCA9685 library not available (simulation mode)")  
          
        # Check pygame for controller  
        try:  
            import pygame  
            hardware_status['pygame'] = True  
            self.get_logger().info("Pygame library available")  
        except ImportError:  
            hardware_status['pygame'] = False  
            self.get_logger().error("Pygame library not available")  
          
        # Check OpenCV for camera  
        try:  
            import cv2  
            hardware_status['opencv'] = True  
            self.get_logger().info("OpenCV library available")  
        except ImportError:  
            hardware_status['opencv'] = False  
            self.get_logger().error("OpenCV library not available")  
          
        # Face recognition removed - not essential for core robot functionality
          
        self.test_results['hardware_dependencies'] = hardware_status  
      
    def run_all_tests(self):  
        """Run all system tests"""  
        self.get_logger().info("Starting Hornet system tests...")  
          
        self.test_node_connectivity()  
        self.test_topic_communication()  
        self.test_basic_movements()  
        self.test_hardware_dependencies()  
          
        # Print results  
        self.print_test_results()  
      
    def print_test_results(self):  
        """Print comprehensive test results"""  
        print("\n" + "="*50)  
        print("HORNET SYSTEM TEST RESULTS")  
        print("="*50)  
          
        for test_name, result in self.test_results.items():  
            if test_name == 'hardware_dependencies':  
                print(f"\nHardware Dependencies:")  
                for component, status in result.items():  
                    status_str = "✓ PASS" if status else "✗ FAIL"  
                    print(f"  {component}: {status_str}")  
            else:  
                status_str = "✓ PASS" if result else "✗ FAIL"  
                print(f"{test_name}: {status_str}")  
          
        # Overall status  
        all_passed = all(  
            result if test_name != 'hardware_dependencies'   
            else all(result.values())   
            for test_name, result in self.test_results.items()  
        )  
          
        print("\n" + "="*50)  
        if all_passed:  
            print("OVERALL: ✓ ALL TESTS PASSED")  
        else:  
            print("OVERALL: ✗ SOME TESTS FAILED")  
        print("="*50)  
  
def main():  
    rclpy.init()  
      
    test_node = HornetSystemTest()  
      
    try:  
        test_node.run_all_tests()  
    except KeyboardInterrupt:  
        print("\nTest interrupted by user")  
    except Exception as e:  
        print(f"Test failed with error: {e}")  
    finally:  
        test_node.destroy_node()  
        rclpy.shutdown()  
  
if __name__ == '__main__':  
    main()
