import rclpy
from rclpy.node import Node
from marvelmind_ros2_msgs.msg import HedgePosition
from collections import deque
import time

class HedgehogPosFilterNode(Node):
    def __init__(self):
        super().__init__('hedgehog_pos_filter_node')

        # Subscriber: listen to /hedgehog_pos topic
        self.subscription = self.create_subscription(
            HedgePosition,
            '/hedgehog_pos',
            self.hedgehog_pos_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Publisher: filtered output on /hedgehog_pos_filter topic
        self.publisher = self.create_publisher(HedgePosition, '/hedgehog_pos_filter', 10)

        # Exponential moving average (EMA) parameter:
        # alpha is the smoothing factor [0, 1]: smaller values lead to slower response.
        self.alpha = 0.1

        # Initialize EMA values (will be set with the first message)
        self.filtered_x = None
        self.filtered_y = None
        self.filtered_z = None

        # Configure moving average filter parameters:
        # window_size defines the number of recent EMA values to average.
        self.window_size = 10  # Adjust as needed

        # Deques for storing the EMA outputs before averaging
        self.x_window = deque(maxlen=self.window_size)
        self.y_window = deque(maxlen=self.window_size)
        self.z_window = deque(maxlen=self.window_size)

        # Store the last received message (for flags, etc.)
        self.last_msg = None

        # Timer to publish filtered messages regularly, even if no new raw data arrives.
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publishing at 10 Hz

    def hedgehog_pos_callback(self, msg: HedgePosition):
        # If first message, initialize EMA values with raw measurements.
        if self.filtered_x is None:
            self.filtered_x = msg.x_m
            self.filtered_y = msg.y_m
            self.filtered_z = msg.z_m
        else:
            # Update EMA values:
            self.filtered_x = self.alpha * msg.x_m + (1 - self.alpha) * self.filtered_x
            self.filtered_y = self.alpha * msg.y_m + (1 - self.alpha) * self.filtered_y
            self.filtered_z = self.alpha * msg.z_m + (1 - self.alpha) * self.filtered_z

        # Append the newly computed EMA values to the averaging window.
        self.x_window.append(self.filtered_x)
        self.y_window.append(self.filtered_y)
        self.z_window.append(self.filtered_z)

        # Save the full message (to copy flags, etc.)
        self.last_msg = msg

    def timer_callback(self):
        # Proceed only if we have some data in the window.
        if not self.x_window:
            return

        # Compute the moving average over the stored EMA values.
        avg_x = sum(self.x_window) / len(self.x_window)
        avg_y = sum(self.y_window) / len(self.y_window)
        avg_z = sum(self.z_window) / len(self.z_window)

        # Create a new filtered message.
        filtered_msg = HedgePosition()
        # Set timestamp (in ms)
        filtered_msg.timestamp_ms = int(time.time() * 1000)
        # Use the combined filter output.
        filtered_msg.x_m = avg_x
        filtered_msg.y_m = avg_y
        filtered_msg.z_m = avg_z
        # Pass through additional data, for example, the flag field from the last raw message.
        filtered_msg.flags = self.last_msg.flags if self.last_msg is not None else 0

        # Publish the filtered message.
        self.publisher.publish(filtered_msg)

        # Log the filtered output in the desired format.
        # self.get_logger().info(
        #     f"timestamp_ms: {filtered_msg.timestamp_ms}\n"
        #     f"x_m: {filtered_msg.x_m:.3f}\n"
        #     f"y_m: {filtered_msg.y_m:.3f}\n"
        #     f"z_m: {filtered_msg.z_m:.3f}\n"
        #     f"flags: {filtered_msg.flags}\n---"
        # )

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
