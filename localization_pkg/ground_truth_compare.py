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

        self.ekf_x0 = None
        self.ekf_y0 = None
        self.gt_x0 = None
        self.gt_y0 = None
        self.initialized = False

        self.errors = []
        self.gps_noise_std = 0.03
        self.imu_noise_std = 0.01

        self.create_subscription(
            Odometry, '/odometry/filtered', self.ekf_cb, 10)
        self.create_subscription(
            ModelStates, '/model_states', self.gt_cb, 10)

        self.create_timer(1.0, self.compute_error)
        self.get_logger().info('Ground Truth Karşılaştırma Node başlatıldı')

    def ekf_cb(self, msg):
        self.ekf_x = msg.pose.pose.position.x
        self.ekf_y = msg.pose.pose.position.y

    def gt_cb(self, msg):
        if 'robotaxi' in msg.name:
            idx = msg.name.index('robotaxi')
            self.gt_x = msg.pose[idx].position.x
            self.gt_y = msg.pose[idx].position.y

    def compute_error(self):
        if None in (self.ekf_x, self.ekf_y, self.gt_x, self.gt_y):
            return

        if not self.initialized:
            self.ekf_x0 = self.ekf_x
            self.ekf_y0 = self.ekf_y
            self.gt_x0 = self.gt_x
            self.gt_y0 = self.gt_y
            self.initialized = True
            self.get_logger().info('Başlangıç noktası kaydedildi')
            return

        ekf_dx = self.ekf_x - self.ekf_x0
        ekf_dy = self.ekf_y - self.ekf_y0
        gt_dx = self.gt_x - self.gt_x0
        gt_dy = self.gt_y - self.gt_y0

        ekf_dx += np.random.normal(0, self.imu_noise_std)
        ekf_dy += np.random.normal(0, self.imu_noise_std)
        gt_dx += np.random.normal(0, self.gps_noise_std)
        gt_dy += np.random.normal(0, self.gps_noise_std)

        error = math.sqrt((ekf_dx - gt_dx)**2 + (ekf_dy - gt_dy)**2)
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
