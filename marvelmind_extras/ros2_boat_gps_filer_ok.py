import rclpy
from rclpy.node import Node
from marvelmind_ros2_msgs.msg import HedgePosition
import time

class HedgehogPosFilterNode(Node):
    def __init__(self):
        super().__init__('hedgehog_pos_filter_node')

        # Subscriber: listen to /hedgehog_pos
        self.subscription = self.create_subscription(
            HedgePosition,
            '/hedgehog_pos',
            self.hedgehog_pos_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher: filtered output on /hedgehog_pos_filter
        self.publisher = self.create_publisher(HedgePosition, '/hedgehog_pos_filter', 10)

        # Filtering parameters:
        # alpha is the smoothing factor [0.0, 1.0]: smaller values cause a slower reaction to new measurements.
        self.alpha = 0.05  # adjust as needed for spike reduction
        self.filtered_x = None
        self.filtered_y = None
        self.filtered_z = None

        # Store the last received message (e.g., for flag values)
        self.last_msg = None

        # Create a timer to publish the filtered message regularly.
        # This guarantees we keep sending values even if no new messages are received.
        self.timer = self.create_timer(0.1, self.timer_callback)  # publishing at 10 Hz

    def hedgehog_pos_callback(self, msg: HedgePosition):
        # On the first message, initialize the filtered values
        if self.filtered_x is None:
            self.filtered_x = msg.x_m
            self.filtered_y = msg.y_m
            self.filtered_z = msg.z_m
        else:
            # Apply an exponential moving average filter to each coordinate
            self.filtered_x = self.alpha * msg.x_m + (1 - self.alpha) * self.filtered_x
            self.filtered_y = self.alpha * msg.y_m + (1 - self.alpha) * self.filtered_y
            self.filtered_z = self.alpha * msg.z_m + (1 - self.alpha) * self.filtered_z

        # Save the last received message (for flags and such)
        self.last_msg = msg

    def timer_callback(self):
        # Do nothing until we have received at least one message.
        if self.filtered_x is None:
            return

        # Create a new filtered message to publish.
        filtered_msg = HedgePosition()

        # Update the timestamp to the current time (in ms).
        filtered_msg.timestamp_ms = int(time.time() * 1000)

        # Assign the filtered values.
        filtered_msg.x_m = self.filtered_x
        filtered_msg.y_m = self.filtered_y
        filtered_msg.z_m = self.filtered_z

        # Pass through any additional data; here we copy the flags from the last received message.
        filtered_msg.flags = self.last_msg.flags if self.last_msg is not None else 0

        # Publish the filtered message.
        self.publisher.publish(filtered_msg)

        # Optionally, also log the filtered output in the desired format.
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
        pass  # Allow clean shutdown on Ctrl+C
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
