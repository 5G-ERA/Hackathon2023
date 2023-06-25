import rclpy
from rclpy.node import Node
import threading
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from era_5g_client.client_base import NetAppClientBase, NetAppLocation

# Will print additional info if True
DEBUG = False

class Teleop_client_client(Node):
    """Node which subscribes the topic of LaserScan type and detects
    the nearest object in front of the sensor. 
    """

    def __init__(self):

        super().__init__('teleop')
        
        # publisher of movement commands to the robot
        self.movement_pub = self.create_publisher(
            Twist,
            "/cmd_vel", 
            10
        )
        
        # client.py network application client instance  
        # Create the client library instance 
        self.client = NetAppClientBase(self.results) 

        # Register with the deployed network application and pass the arguments
        self.client.register(NetAppLocation("localhost", 5896))
        
    def results(self, data):     
        
        if "result" not in data:
            return
        print('publish cmd_vel...')
        twist = Twist()
        cmd_dict = data['result']
        twist.linear.x = cmd_dict["linear"]["x"]
        twist.linear.y = cmd_dict["linear"]["y"]
        twist.linear.z = cmd_dict["linear"]["z"]
        twist.angular.x = cmd_dict["angular"]["x"]
        twist.angular.y = cmd_dict["angular"]["y"]
        twist.angular.z = cmd_dict["angular"]["z"]
        
        self.movement_pub.publish(twist)
    

def main():
    rclpy.init()

    teleop_node = Teleop_client_client(90) # 90 degrees
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(teleop_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
