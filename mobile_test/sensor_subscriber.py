import rclpy as rp
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class SensorSubscriber(Node):

    def __init__(self):
        super().__init__('sensor_subscriber')
        self.imu_data = self.create_subscription(
            Imu, '/imu_plugin/out', self.imu_callback,10
        )
        self.odom_data = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

    def imu_callback(self, msg):
        print("\n\nLinear_acceleration: \n","x: ", msg.linear_acceleration.x, "y: ", msg.linear_acceleration.y, "z: ", msg.linear_acceleration.z, 
              "\nAngular_velocity: \n","ax: ", msg.angular_velocity.x, "ay: ", msg.angular_velocity.y, "az: ", msg.angular_velocity.z)    

    def odom_callback(self, msg):
        print("\n\nPose: \n", "- position: \n" "xyz: ", [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
              "\n- Quaternion: \n", "quat xyz: ", [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z],
              "\n\nTwist: \n", "- linear: \n", "vel xyz: ", [msg.twist.twist.linear.x,msg.twist.twist.linear.y, msg.twist.twist.linear.z],
              "- angular: \n", "ang xyz: ", [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])

def main(args=None):
    rp.init(args=args)

    sensor_subscriber = SensorSubscriber()
    rp.spin(sensor_subscriber)
    sensor_subscriber.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()