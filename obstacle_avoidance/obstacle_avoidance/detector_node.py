import rclpy
from rclpy.node import Node
import threading
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

# Will print additional info if True
DEBUG = False


class NearestObstacle(Node):
    """Node which subscribes the topic of LaserScan type and detects
    the nearest object in front of the sensor. 

    """

    def __init__(self, scan_width: int):
        """Constructor

        Args:
            scan_width (int): The angle (degrees) of the sector in which the object 
                              should be detected.
        """
        super().__init__('nearest_obstacle')
        # subscriber of the laser data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback, # callback called once new data arrives
            1)
        
        # publisher which provides the distance (in meters) to the nearest obstacle
        self.distance_pub = self.create_publisher(
            Float32,
            "/nearest_obstacle", 
            10
        )
        self.scan_width = scan_width * 3.1415 / 180

    def listener_callback(self, msg: LaserScan):
        """Callback called once new data arrives.

        Args:
            msg (LaserScan): The LaserScan message.
        """

        if DEBUG:
            self.get_logger().info(f"min: {msg.angle_min}, max: {msg.angle_max}, inc: {msg.angle_increment}, cnt: {len(msg.ranges)}")
        
        # check if the set scan_width is lower than the actual with of the data from the sensor
        if abs(len(msg.ranges) * msg.angle_increment) < self.scan_width:
            self.get_logger().error("The requested width of scan is bigger then the scanner width!")
            return
        
        ################################################
        #### The "core algorithm" starts here       ####
        ################################################

        # The documentation for the LaserScan message:
        # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html

        # The "ranges" field contains distances (in meters) to nearest object, starting with
        # "angle_min" angle and finishing with angle_max angle, while the angle between two
        # consecutive values is "angle_increment"

        # the index of the "middle" value, i.e., the front of the robot (in ideal case).
        middle = int(len(msg.ranges) / 2)
        
        # the amount of values in ranges array to both sides from the middle
        width = int((self.scan_width / 2) / abs(msg.angle_increment))
        
        # the section of ranges in selected width
        ranges = msg.ranges[middle - width : middle + width]

        # the "nearest" obstacle in front of the robot
        print(min(ranges))
        
        ################################################
        #### The "core algorithm" ends here         ####
        ################################################

        distance = Float32()
        distance.data = min(ranges)
        self.distance_pub.publish(distance)


class Driver(Node):
    """The node which "drives" the robot around. When no obstacle is detected,
    the robot drives forward. Once the obstacle is detected in certain distance,
    the robot starts rotating until the path is empty.
    """

    def __init__(self, safe_zone: int):
        """The constructor.

        Args:
            safe_zone (int): The distance to nearest obstacle, which is considered safe to 
                             move the robot forward.
        """
        super().__init__('driver')

        # the subscriber of the nearest obstacle topic
        self.subscription = self.create_subscription(
            Float32,
            '/nearest_obstacle',
            self.listener_callback,
            1)
        
        # the latest distance to the nearest obstacle
        self.nearest_obstacle = 0
        # the velocity command publisher
        self.speed_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.safe_zone = safe_zone
        # how ofthen is the robot's velocity updated
        timer_period = 0.05  # seconds
        self.publish_speed_timer = self.create_timer(timer_period, self.publish_speed)

    def listener_callback(self, msg: Float32):
        # update the latest distance to the nearest obstacle
        self.nearest_obstacle = msg.data

    def publish_speed(self): 
        speed = Twist()
        # if the nearest obstacle is outside the safe zone, drives forward
        if self.nearest_obstacle > self.safe_zone:
            speed.linear.x = 0.1
        else: # otherwise rotate until the obstacle is not in front of the robot
            speed.angular.z = 0.7
        self.speed_publisher.publish(speed)
        


def main():
    rclpy.init()

    nearest_obstacle = NearestObstacle(90) # 90 degrees
    driver = Driver(0.2) # 20cm
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(nearest_obstacle)
    executor.add_node(driver)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = driver.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    nearest_obstacle.destroy_node()
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
