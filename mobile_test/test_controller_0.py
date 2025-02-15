import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time

class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.angle = 0.0

    def update(self, gyro_rate, accel_angle, dt):
        self.angle = self.alpha * (self.angle + gyro_rate * dt) + \
                     (1 - self.alpha) * accel_angle
        return self.angle

def quaternion_to_euler(q):
    t0 = 2.0 * (q.w * q.x + q.y * q.z)
    t1 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (q.w * q.y - q.z * q.x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = 2.0 * (q.w * q.z + q.x * q.y)
    t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        self.imu_sub = self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cf = ComplementaryFilter()
        self.last_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

        self.angular_velocity = None
        self.linear_acceleration = None
        self.position = None
        self.orientation = None
        self.linear_velocity = None

        self.accumulated_distance = 0.0
        self.is_rotating = False
        self.rotation_start_time = None
        self.target_distance = 1.2  # 목표 이동 거리
        self.rotation_duration = .6  # 회전에 걸리는 시간 (초)

    def imu_callback(self, msg):
        self.angular_velocity = msg.angular_velocity
        self.linear_acceleration = msg.linear_acceleration

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.linear_velocity = msg.twist.twist.linear

    def update(self):
        if not all([self.angular_velocity, self.linear_acceleration, self.position, self.orientation]):
            self.get_logger().warn('Waiting for sensor data...')
            return

        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        dt = current_time - self.last_time
        self.last_time = current_time

        accel_angle = math.atan2(self.linear_acceleration.y, self.linear_acceleration.z)
        gyro_rate = self.angular_velocity.x

        filtered_angle = self.cf.update(gyro_rate, accel_angle, dt)
        self.get_logger().info(f'filtered_angle : {filtered_angle}')
        _, _, yaw = quaternion_to_euler(self.orientation)
        self.get_logger().info(f'yaw : {yaw}')

        cmd_vel = Twist()
        

        if self.is_rotating:
            if current_time - self.rotation_start_time > self.rotation_duration:
                self.is_rotating = False
                self.accumulated_distance = 0.0
                self.get_logger().info("Rotation completed, resuming forward motion")
            else:
                cmd_vel.angular.z = math.pi / 2
                self.get_logger().info(f'first cmd_vel.angular.z : {cmd_vel.angular.z}')
        else:

            cmd_vel.linear.x = 0.2  # 전진 속도
            #cmd_vel.angular.z = 0.1 * (filtered_angle - yaw)  # 각도에 따른 회전
            cmd_vel.angular.z = 0.

            self.accumulated_distance += cmd_vel.linear.x * dt
            if self.accumulated_distance >= self.target_distance:
                self.is_rotating = True
                self.rotation_start_time = current_time
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.get_logger().info("Target distance reached, starting rotation")
                

        self.get_logger().info(f'Publishing cmd_vel: linear.x={cmd_vel.linear.x}, angular.z={cmd_vel.angular.z}')
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
            node.update()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
