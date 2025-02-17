import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import math
import numpy as np

class SensorFusion:
    def __init__(self):
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.zeros(3)
        self.P = np.eye(9)  # 상태 공분산 행렬
        self.Q = np.eye(9) * 0.1  # 프로세스 노이즈 공분산
        self.R = np.eye(6) * 0.7  # 측정 노이즈 공분산, 노이즈 값 크게 잡고 테스트 진행

    def predict(self, accel, gyro, dt):
        # imu 센서 이용
        # 퓨전하려면 활용해야 함, 현재 활용x
        # 간단한 운동 모델을 사용한 예측
        self.position += self.velocity * dt + 0.5 * accel * dt**2
        self.velocity += accel * dt
        self.orientation += gyro * dt

        # 자코비안 행렬
        F = np.eye(9)
        F[0:3, 3:6] = np.eye(3) * dt
        
        # 공분산 업데이트
        self.P = F @ self.P @ F.T + self.Q

    def update(self, odom_pos, odom_ori):
        # odom data를 활용한 state 업데이트

        H = np.zeros((6, 9))
        H[0:3, 0:3] = np.eye(3)  # 위치에 대한 측정 모델
        H[3:6, 6:9] = np.eye(3)  # 방향에 대한 측정 모델

        z = np.concatenate([odom_pos, odom_ori])
        y = z - H @ np.concatenate([self.position,np.zeros(3), self.orientation])

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        state_update = K @ y
        self.position += state_update[0:3]
        self.orientation += state_update[6:9]

        self.P = (np.eye(9) - K @ H) @ self.P

        
class SensorFusionNode(Node):
    # robot controller 
    # cmd_vel로 인풋 -> 조건별 분기로 사각 궤적
    # 클래스 변수로 node 안에서 반복시 초기화되지 않도록
    count = 1
    time = 0.
    change = 'go'

    def __init__(self):
        super().__init__('sensor_fusion_node')
        self.imu_sub = self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.fusion = SensorFusion()
        self.last_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

        # 센서 데이터 초기화
        self.accel = None
        self.gyro = None
        self.odom_pos = None
        self.odom_ori = None


        # robot's initial state & goal
        self.state = 'moving'
        self.target_position = np.array([1., 0., 0.])
        self.target_orientation = 0.0


    def imu_callback(self, msg):
        self.accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

    def odom_callback(self, msg):
        self.odom_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        _, _, yaw = self.quaternion_to_euler(msg.pose.pose.orientation)
        self.odom_ori = np.array([0, 0, yaw])  # yaw만 사용

    def quaternion_to_euler(self, q):
        # quaternion_to_euler 함수 사용
        t0 = 2. * (q.w * q.x + q.y * q.z)
        t1 = 1. - 2. * (q.x * q.x + q.y + q.y)
        roll = math.atan2(t0,t1)

        t2 = 2. * (q.w * q.y - q.z * q.x)
        t2 = 1.0 if t2 > 1. else t2
        t2 = -1.0 if t2 < -1. else t2
        pitch = math.asin(t2) 
        
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def update(self):
        # robot square trajectory controller 
        if self.accel is None or self.gyro is None or self.odom_pos is None or self.odom_ori is None:
            self.get_logger().warn('Waiting for sensor data...')
            return

        # predict를 통해 센서 퓨전시 활용할 dt
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        dt = current_time - self.last_time
        self.last_time = current_time

        self.fusion.predict(self.accel, self.gyro, dt)
        self.fusion.update(self.odom_pos, self.odom_ori)


        # error of directions and positions
        position_error = self.target_position - self.fusion.position
        # orientation_error = self.target_orientation - self.fusion.orientation[2]  # Assuming yaw is the last element
        # 2개의 라인 코드가 같은 결과를 보이는 것 같지만 atan2를 쓰면 -pi ~ pi까지 정규화를 해준다.
        orientation_error = math.atan2(math.sin(self.target_orientation - self.fusion.orientation[2]),
                               math.cos(self.target_orientation - self.fusion.orientation[2]))
        self.get_logger().info(f'position_error: {position_error}')
        self.get_logger().info(f'oritentation: {orientation_error}')

        cmd_vel = Twist()

        # Move & Rotation control
        # phase 1: moving > rotating
        # phase 2: moving
        # phase 3: 각도 error 작을 때
        # phase 4: rotating > moving
        # phase 5: 생략 가능, ori 오차 더 작게 설정
        # phase 6: 목표 달성 다음 목표 업데이트
        # phase 7: ori 오차 줄이는 과정
        if self.state == 'moving':
            if np.linalg.norm(position_error[:2]) < 0.1:
                self.state = 'rotating'
                self.get_logger().info('phase 1')
                
            else:
               
                cmd_vel.linear.x = 0.35
                cmd_vel.angular.z = 0.6 * orientation_error
                self.get_logger().info(f'fusion: {self.fusion.orientation[2]}')
                self.get_logger().info('phase 2')    
                
        elif self.state == 'rotating':
            cmd_vel.linear.x = 0.
            if abs(orientation_error) < 0.1:
                self.get_logger().info('rot phase 3')
                if SensorFusionNode.change == 'stop':
                   
                    self.get_logger().info('rot phase 4')
                    #if abs(orientation_error) < 0.02: # 해당 부분은 에러 허용치를 낮추는 라인이라 굳이 필요 x
                     #self.get_logger().info('rot phase 5')
                    cmd_vel.angular.z = 0.0
                    self.state = 'moving'
                    SensorFusionNode.change = 'go'
                else:
                    SensorFusionNode.count += 1
                    self.update_target_position()
                    SensorFusionNode.change = 'stop'
                    self.get_logger().info(f'rot phase 6: {self.target_position}')
                    self.get_logger().info(f'rot phase 6: {self.target_orientation}')

                    

            else:
                cmd_vel.angular.z = 0.5 * orientation_error  # Adjust rotation speed as needed   
                self.get_logger().info('rot phase 7')   
                
        
  
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info(f'Publishing cmd_vel: linear.x={cmd_vel.linear.x}, angular.z={cmd_vel.angular.z}')


    def update_target_position(self):
        # 4 Case for square trajectory
        if self.count % 4 == 1:
            self.target_position = np.array([1., 0., 0.])
            self.target_orientation = - 0.01
            self.get_logger().info('T1')
        elif self.count % 4 == 2:
            self.target_position = np.array([1., 1., 0.])
            self.target_orientation = math.pi/2
            self.get_logger().info('T2') 
        elif self.count % 4 == 3:
            self.target_position = np.array([0., 1., 0.])
            self.target_orientation = math.pi -0.01  # atan2 는 -pi ~ pi 범위
            self.get_logger().info('T3')
        else:
            self.target_position = np.array([0.0, 0.0, 0.])
            self.target_orientation = - math.pi /2
            self.get_logger().info('T4')
                    

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
