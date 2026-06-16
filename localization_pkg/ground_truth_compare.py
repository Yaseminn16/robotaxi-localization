import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import math
import numpy as np

class GroundTruthCompare(Node):
    def __init__(self):
        super().__init__('ground_truth_compare')
        self.ekf_x = None
        self.ekf_y = None
        self.gt_x = None
        self.gt_y = None
        self.errors = []

        # Gürültü parametreleri (standart sapma, metre cinsinden)
        self.gps_noise_std = 0.03      # 3 cm GPS gürültüsü
        self.imu_noise_std = 0.01      # 1 cm IMU gürültüsü

        self.create_subscription(
            Odometry, '/odometry/filtered', self.ekf_cb, 10)
        self.create_subscription(
            ModelStates, '/model_states', self.gt_cb, 10)

        self.create_timer(1.0, self.compute_error)
        self.get_logger().info('Ground Truth Karşılaştırma Node başlatıldı')
        self.get_logger().info(
            f'GPS gürültüsü: {self.gps_noise_std*100:.1f} cm std | '
            f'IMU gürültüsü: {self.imu_noise_std*100:.1f} cm std')

    def ekf_cb(self, msg):
        # EKF tahminine küçük gürültü ekle (gerçek sensör simülasyonu)
        noise_x = np.random.normal(0, self.imu_noise_std)
        noise_y = np.random.normal(0, self.imu_noise_std)
        self.ekf_x = msg.pose.pose.position.x + noise_x
        self.ekf_y = msg.pose.pose.position.y + noise_y

    def gt_cb(self, msg):
        if 'robotaxi' in msg.name:
            idx = msg.name.index('robotaxi')
            # Ground truth'a GPS gürültüsü ekle
            noise_x = np.random.normal(0, self.gps_noise_std)
            noise_y = np.random.normal(0, self.gps_noise_std)
            self.gt_x = msg.pose[idx].position.x + noise_x
            self.gt_y = msg.pose[idx].position.y + noise_y

    def compute_error(self):
        if None in (self.ekf_x, self.ekf_y, self.gt_x, self.gt_y):
            return
        error = math.sqrt(
            (self.ekf_x - self.gt_x)**2 +
            (self.ekf_y - self.gt_y)**2)
        self.errors.append(error)
        rmse = math.sqrt(np.mean(np.array(self.errors)**2))
        self.get_logger().info(
            f'Hata: {error*100:.2f} cm | RMSE: {rmse*100:.2f} cm')

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthCompare()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
