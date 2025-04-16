import rclpy
from rclpy.node import Node
from marvelmind_ros2_msgs.msg import HedgePosition
from std_msgs.msg import Float32
from collections import deque
import time

class HedgehogPosFilterNode(Node):
    def __init__(self):
        super().__init__('hedgehog_pos_filter_node')


        self.subscription = self.create_subscription(
            HedgePosition,
            '/hedgehog_pos',
            self.hedgehog_pos_callback,
            10
        )
        self.subscription  # 

       
        self.publisher = self.create_publisher(HedgePosition, '/hedgehog_pos_filtered', 10)

        ###############
        ###############
        ###############
        self.alpha = 0.1

        # Initialize EMA values for Hedgehog (will be set with the first message).
        self.filtered_x = None
        self.filtered_y = None
        self.filtered_z = None

        ###############
        ###############
        ###############
        self.window_size = 10  # Adjust as needed.

        # Deques for storing the EMA outputs before averaging for Hedgehog.
        self.x_window = deque(maxlen=self.window_size)
        self.y_window = deque(maxlen=self.window_size)
        self.z_window = deque(maxlen=self.window_size)

        # Store the last received Hedgehog message (for flags, etc.).
        self.last_msg = None

        # --------------------------------------------

        self.imu_subscription = self.create_subscription(
            Float32,
            '/bluebot/imu_compass_fused',
            self.imu_callback,
            10
        )
        self.imu_subscription  # prevent unused variable warning

        # Publisher for the filtered IMU compass data on /bluebot/imu_compass_fused_filtered.
        self.imu_pub = self.create_publisher(Float32, '/imu_compass_fused_filtered', 10)

        # Exponential moving average parameter for IMU compass data.
        self.imu_alpha = 0.05  # Adjust as needed.

        # Initialize EMA value for the IMU compass.
        self.imu_filtered = None

        ###############
        ###############
        ###############.
        self.imu_window_size = 20  # Number of recent EMA values to average.
        self.imu_window = deque(maxlen=self.imu_window_size)

        ###############
        ###############
        ###############
        self.imu_offset = 70.0  # Can be adjusted as needed.

        # --------------------------------------------

        self.timer = self.create_timer(0.1, self.timer_callback)  # Publishing at 10 Hz

    def hedgehog_pos_callback(self, msg: HedgePosition):
        # Initialize or update EMA for Hedgehog positions.
        if self.filtered_x is None:
            self.filtered_x = msg.x_m
            self.filtered_y = msg.y_m
            self.filtered_z = msg.z_m
        else:
            self.filtered_x = self.alpha * msg.x_m + (1 - self.alpha) * self.filtered_x
            self.filtered_y = self.alpha * msg.y_m + (1 - self.alpha) * self.filtered_y
            self.filtered_z = self.alpha * msg.z_m + (1 - self.alpha) * self.filtered_z

        # Append the updated EMA values to the respective averaging windows.
        self.x_window.append(self.filtered_x)
        self.y_window.append(self.filtered_y)
        self.z_window.append(self.filtered_z)

        # Save the full message (to copy flags, etc.).
        self.last_msg = msg

    def imu_callback(self, msg: Float32):
        # Initialize or update EMA for IMU compass data.
        if self.imu_filtered is None:
            self.imu_filtered = msg.data
        else:
            self.imu_filtered = self.imu_alpha * msg.data + (1 - self.imu_alpha) * self.imu_filtered

        # Append the updated EMA value to the IMU averaging window.
        self.imu_window.append(self.imu_filtered)

    def timer_callback(self):
        # --------------------------
        # Publish filtered Hedgehog position.
        if self.x_window:
            # Compute the moving average over the stored EMA values.
            avg_x = sum(self.x_window) / len(self.x_window)
            avg_y = sum(self.y_window) / len(self.y_window)
            avg_z = sum(self.z_window) / len(self.z_window)

            # Create and populate the filtered Hedgehog message.
            filtered_msg = HedgePosition()
            filtered_msg.timestamp_ms = int(time.time() * 1000)
            filtered_msg.x_m = avg_x
            filtered_msg.y_m = avg_y
            filtered_msg.z_m = avg_z
            filtered_msg.flags = self.last_msg.flags if self.last_msg is not None else 0

            # Publish the filtered Hedgehog position.
            self.publisher.publish(filtered_msg)
            # Optionally, log the output.
            # self.get_logger().info(f"Hedgehog Filtered: x: {avg_x:.3f}, y: {avg_y:.3f}, z: {avg_z:.3f}")

        # --------------------------
        # Publish filtered IMU compass value.
        if self.imu_window:
            # Compute the moving average over the stored EMA values.
            avg_imu = sum(self.imu_window) / len(self.imu_window)
            # Add the configurable offset.
            filtered_imu = avg_imu + self.imu_offset

            # Create the filtered IMU compass message.
            imu_msg = Float32()
            imu_msg.data = filtered_imu

            # Publish the filtered IMU compass value.
            self.imu_pub.publish(imu_msg)
            # Optionally, log the IMU filter output.
            # self.get_logger().info(f"IMU Filtered: {filtered_imu:.3f} (offset: {self.imu_offset})")

def main(args=None):
    rclpy.init(args=args)
    node = HedgehogPosFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Clean shutdown on Ctrl+C
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
