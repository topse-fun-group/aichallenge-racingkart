#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from v2x_msgs.msg import V2XVehiclePositionArray, V2XVehiclePosition

class V2XDummyPublisher(Node):
    """自車位置の前方に仮想の低速他車を自動生成・パブリッシュするテスト用ノード"""
    def __init__(self):
        super().__init__('v2x_dummy_publisher')
        self._odom_sub = self.create_subscription(
            Odometry, '/localization/kinematic_state', self._odom_cb, 10)
        self._v2x_pub = self.create_publisher(
            V2XVehiclePositionArray, '/v2x/vehicle_positions', 10)
        
        self._timer = self.create_timer(0.1, self._publish_cb) # 10Hz
        
        self._ego_x = 0.0
        self._ego_y = 0.0
        self._ego_yaw = 0.0
        self._ego_speed = 0.0
        self._has_odom = False
        
        # 仮想他車パラメータ
        self._distance_ahead = 10.0 # [m] 自車前方10mに配置
        self._dummy_speed = 5.55    # [m/s] 約 20.0 km/h
        
        self.get_logger().info(
            "[V2X Dummy Publisher] Running! Simulating leading car 10m ahead at 20km/h..."
        )

    def _odom_cb(self, msg: Odometry):
        self._ego_x = msg.pose.pose.position.x
        self._ego_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self._ego_yaw = math.atan2(siny_cosp, cosy_cosp)
        self._ego_speed = msg.twist.twist.linear.x
        self._has_odom = True

    def _publish_cb(self):
        if not self._has_odom:
            return

        # 自車の進行方向10m前方に仮想車両を計算配置
        target_x = self._ego_x + self._distance_ahead * math.cos(self._ego_yaw)
        target_y = self._ego_y + self._distance_ahead * math.sin(self._ego_yaw)

        v2x_array = V2XVehiclePositionArray()
        v2x_array.header.stamp = self.get_clock().now().to_msg()
        v2x_array.header.frame_id = 'map'

        dummy_car = V2XVehiclePosition()
        dummy_car.header.stamp = self.get_clock().now().to_msg()
        dummy_car.header.frame_id = 'map'
        dummy_car.vehicle_id = 'dummy_leading_kart'
        dummy_car.position.x = target_x
        dummy_car.position.y = target_y
        dummy_car.position.z = 0.0

        v2x_array.vehicles.append(dummy_car)
        self._v2x_pub.publish(v2x_array)

def main(args=None):
    rclpy.init(args=args)
    node = V2XDummyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
