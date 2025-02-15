import math

class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.angle = 0.0

    def update(self, gyro_rate, accel_angle, dt):
        self.angle = self.alpha * (self.angle + gyro_rate * dt)+
        (1 - self.alpha) * accel_angle
        return self.angle
    
    def quaternion_to_euler(q):
        t0 = 2. * (q.w * q.x + q.y * q.z)
        t1 = 1. - 2. * (q.x * q.x + q.y + q.y)
        roll = math.atan2(t0,t1)

        t2 = 2. * (q.w * q.y - q.z * q.x)
        t2 = 1.0 if t2 > 1. else t2
        t2 = -1.0 if t2 < -1. else t2
        pitch = math.atan2(t2) 
        
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw