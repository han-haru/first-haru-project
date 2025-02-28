import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np

class SensorFusion:
    def __init__(self):
        self.position = np.zeros(2)
        self.velocity = np.zeros(3)
        self.orientation = np.zeros(1)
        self.P = np.eye(6)  # 상태 추정 오차 공분산 행렬
        self.Q = np.array(
            [[0.001507,  0.0006,   -0.024654,  0.000188,  0.,        0.000955]
[ 0.0006,    0.000995, -0.021857,  0.001101,  0.,       -0.00315 ]
[-0.024654, -0.021857,  0.775711, -0.021959,  0.,        0.103542]
[ 0.000188,  0.001101, -0.021959,  0.002323,  0.,       -0.003333]
[ 0.,        0.,        0.,        0.,        0.,        0.      ]
[ 0.000955, -0.00315,   0.103542, -0.003333,  0.,        0.223185]])  # 프로세스 노이즈 공분산
        self.R = np.array(
            [[ 0.001,-0.   ,-0.018],
[-0.   , 0.   ,-0.002],
[-0.018,-0.002, 0.814]]
        )  # 측정 노이즈 공분산, 값이 크면 측정값 불신, 작으면 과신
        # self.imu_data = []
        # self.odom_data = []

    def predict(self, accel, gyro, dt):
        # 간단한 운동 모델을 사용한 예측
        # x(t|t-1), P(t|t-1)
        self.position += self.velocity * dt + 0.5 * accel * dt**2
        self.velocity += accel * dt
        self.orientation += gyro * dt

        # 자코비안 행렬
        # 바퀴의 회전 속도를 로봇의 선속도와 각속도로 변환할 때 사용된다.
        # 만약 역기구학 사용하면, 목표 위치에 가기 위한 바퀴 회전 속도를 계산할 때도 사용가능하다.
        F = np.eye(6)
        F[0:3, 3:6] = np.eye(3) * dt
        
        # 공분산 업데이트
        self.P = F @ self.P @ F.T + self.Q

    def update(self, odom_pos, odom_ori):
        # odom data를 활용한 state 업데이트
        # x(t|t), P(t|t), K(t), H(t), R(t)
        # 관측 행렬
        H = np.zeros((3, 6))
        H[0:2, 0:2] = np.eye(2)  # 위치에 대한 측정 모델
        H[2, 2] = np.eye(1)  # 방향에 대한 측정 모델
        H[0:2, 3:6] = np.eye(3) # 차원 채워주는 역할

        z = np.concatenate([odom_pos, odom_ori]) # odom 데이터 통합 [odom_pos odom_ori] : 1차원 벡터 (3,)

        # 측정 잔차(실제 관측과 예측 모델 사이 차이)
        # odom 데이터와 관측 모델 사이 오차 관측 모델에서 특정 값 가져온 후 이를 odom data에서 뺌
        y = z - H @ np.concatenate([self.position, self.orientation ,np.zeros(3)]) # y: (3,), z: (3,), [H:(3,6) @ (6,)]: (3,)

        S = H @ self.P @ H.T + self.R # S: (3,3), [H: (3,6), P: (6,6), H.T: (6,3)]: (3,3), R:(3,3)
        # kalman gain
        # 예측값과 측정값 사이 가중치 설정, 값이 크면 측정값을 값이 작으면 예측값을 신뢰
        K = self.P @ H.T @ np.linalg.inv(S) # K: (6,3), P: (6,6), H.T: (6,3), S: (3,3)

        state_update = K @ y # [(6,3) @ (3,)]: (6,) = [x,y,yaw,0,0,0]
        self.position += state_update[0:2]
        self.orientation += state_update[2]

        self.P = (np.eye(6) - K @ H) @ self.P
    
    # def collect_data(self, accel, gyro, odom_pos, odom_ori):
    #     # collect sensor data for covariance matirx
    #     self.imu_data.append(np.concatenate([accel, gyro]))
    #     self.odom_data.append([odom_pos,odom_ori])

    # def calculate_process_covariance(self):
    #     if len(self.imu_data) > 100 :
    #         imu_cov = np.cov(np.array(self.imu_data).T)
    #         self.Q = imu_cov
            
    #         # 데이터 초기화
    #         self.imu_data = []
    #         self.odom_data = []

    # def calculate_measurement_covariance(self):
    #     if  len(self.odom_data) > 100:
    #         odom_cov = np.cov(np.array(self.odom_data).T)
    #         self.R = odom_cov
            
    #         # 데이터 초기화
    #         self.imu_data = []
    #         self.odom_data = []

class SensorFusionNode(Node):
    # robot controller

    # 클래스 변수로 node 안에서 반복시 초기화되지 않도록
    count = 1
    time = 0.
    change = 'go'
    def __init__(self):
        super().__init__('sensor_fusion_node')
        '''
        # 내장 데이터
        self.imu_sub = self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        '''
        
        #실제 데이터 받을 때
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'motor_topic', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        

        self.fusion = SensorFusion()
        self.last_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

        # 센서 데이터 초기화
        self.accel = None
        self.gyro = None
        self.odom_pos = None
        self.odom_ori = None

        # robot's state & goal
        self.state = 'moving'
        self.target_position = np.array([1., 0.]) # (x, y): (2,)
        self.target_orientation = 0.0 # (y): (1,)

        self.imu_data = []
        self.odom_data = []

    def imu_callback(self, msg):
        self.accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y]) # 1차원 벡터 (2,)
        self.gyro = np.array([msg.angular_velocity.z]) # 1차원 벡터 (1,)

    def odom_callback(self, msg):
        self.odom_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y]) # 1차원 벡터 (2,)
        _, _, yaw = self.quaternion_to_euler(msg.pose.pose.orientation)
        self.odom_ori = np.array([yaw])  # yaw만 사용 1차원 벡터 (1,)



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


        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        dt = current_time - self.last_time
        self.last_time = current_time

        # SensorFusion 클래스 내용받아오기.
        # self.fusion.collect_data(self.accel, self.gyro, self.odom_pos, self.odom_ori)
        # self.fusion.calculate_process_covariance()
        # self.get_logger().info('Covariance matrix of Process updated')
        # self.fusion.calculate_measurement_covariance()
        # self.get_logger().info('Covariance matrix of Measurement updated')
        self.fusion.predict(self.accel, self.gyro, dt)
        self.fusion.update(self.odom_pos, self.odom_ori)


        # error of directions and positions
        position_error = self.target_position - self.fusion.position
        # orientation_error = self.target_orientation - self.fusion.orientation[2]  # Assuming yaw is the last element
        # 2개의 라인 코드가 같은 결과를 보이는 것 같지만 atan2를 쓰면 -pi ~ pi까지 정규화를 해준다.
        orientation_error = math.atan2(math.sin(self.target_orientation - self.fusion.orientation),
                               math.cos(self.target_orientation - self.fusion.orientation))
        #self.get_logger().info(f'position_error: {position_error}')
        #self.get_logger().info(f'oritentation: {orientation_error}')

        cmd_vel = Twist()

        # Move & Rotation control
        if self.state == 'moving':
            if np.linalg.norm(position_error[0:]) < 0.1:
                self.state = 'rotating'
                #self.get_logger().info('phase 1')

            else:
                # 수정된 부분
                cmd_vel.linear.x = 0.15 #np.linalg.norm(position_error) # 남은 거리에 따라 속도 조절
                cmd_vel.angular.z = 0.6 * orientation_error # 남은 각도에 따라 각속도 조절
                #self.get_logger().info(f'fusion: {self.fusion.orientation[2]}')
                #self.get_logger().info('phase 2')    
                
        elif self.state == 'rotating':
            cmd_vel.linear.x = 0.
            if abs(orientation_error) < 0.1:
                #self.get_logger().info('rot phase 3')
                if SensorFusionNode.change == 'stop':
                    #cmd_vel.angular.z = 0.3 * orientation_error
                    #self.get_logger().info('rot phase 4')
                    #if abs(orientation_error) < 0.02: # 해당 부분은 에러 허용치를 낮추는 라인이라 굳이 필요 x
                    cmd_vel.angular.z = 0.0
                    self.state = 'moving'
                    #self.get_logger().info('rot phase 5')
                    SensorFusionNode.change = 'go'
                else:
                    SensorFusionNode.count += 1
                    self.update_target_position()
                    SensorFusionNode.change = 'stop'
                    #self.get_logger().info(f'rot phase 6: {self.target_position}')
                    #self.get_logger().info(f'rot phase 6: {self.target_orientation}')

                    

            else:
                cmd_vel.angular.z = 0.5 * orientation_error  # Adjust rotation speed as needed   
                #self.get_logger().info('rot phase 7')   
                
        
  
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info(f'Publishing cmd_vel: linear.x={cmd_vel.linear.x}, angular.z={cmd_vel.angular.z}')


    def update_target_position(self):
        # 4 Case for square trajectory
        if self.count % 4 == 1:
            self.target_position = np.array([1., 0.])
            self.target_orientation = - 0.01
            self.get_logger().info('T1')
        elif self.count % 4 == 2:
            self.target_position = np.array([1., 1.])
            self.target_orientation = math.pi/2
            self.get_logger().info('T2') 
        elif self.count % 4 == 3:
            self.target_position = np.array([0., 1.])
            self.target_orientation = math.pi -0.01  # atan2 는 -pi ~ pi 범위
            self.get_logger().info('T3')
        else:
            self.target_position = np.array([0.0, 0.0])
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
        pass, 0.
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
