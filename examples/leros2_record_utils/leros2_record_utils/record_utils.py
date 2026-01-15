import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
import sys
import threading
import select
import termios
import tty
from datetime import datetime
import os

from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

class LeROS2RecordUtils(Node):

    def __init__(self):
        super().__init__("leros2_record_utils")
        
        self.declare_parameter("task_name", "test_task")
        self.declare_parameter("button_index", 0)
        self.declare_parameter("topics", ["/lerobot_event", "/lerobot_task"])
        self.declare_parameter("bag_name_prefix", "lerobot_bag")

        self.task_name = self.get_parameter("task_name").get_parameter_value().string_value
        self.button_index = self.get_parameter("button_index").get_parameter_value().integer_value
        self.topics_to_record = self.get_parameter("topics").get_parameter_value().string_array_value
        self.bag_name_prefix = self.get_parameter("bag_name_prefix").get_parameter_value().string_value
        
        self.is_recording_episode = False
        self.is_recording_bag = False
        
        self.writer = None
        self.bag_subs = []

        self._event_publisher = self.create_publisher(String, "lerobot_event", 10)
        self._task_publisher = self.create_publisher(String, "lerobot_task", 10)
        self._is_recording_publisher = self.create_publisher(Bool, "lerobot_is_recording", 10)

        self._joy_subscription = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10
        )
        self._last_button_state = 0

        self.get_logger().info(f"LeROS2 Record Utils Node Started.")
        self.get_logger().info(f"Task Name: {self.task_name}, Button Index: {self.button_index}")
        self.get_logger().info(f"Topics to record: {self.topics_to_record}")
        self.get_logger().info("Controls:")
        self.get_logger().info("  'b': Toggle BAG recording (Start this first!)")
        self.get_logger().info("  'r' or Joy Button: Toggle EPISODE recording")

        # Start keyboard listener in a separate thread
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def joy_callback(self, msg):
        if self.button_index < len(msg.buttons):
            current_button_state = msg.buttons[self.button_index]
            # Check for rising edge
            if current_button_state == 1 and self._last_button_state == 0:
                self.toggle_episode_recording()
            self._last_button_state = current_button_state

    def toggle_episode_recording(self):
        if not self.is_recording_bag:
            self.get_logger().warn("Cannot start EPISODE recording: No active BAG recording. Press 'b' first.")
            return

        if not self.is_recording_episode:
            # Start recording episode
            self.get_logger().info(f"Starting EPISODE recording: {self.task_name}")
            
            task_msg = String()
            task_msg.data = self.task_name
            self._task_publisher.publish(task_msg)

            is_rec_msg = Bool()
            is_rec_msg.data = True
            self._is_recording_publisher.publish(is_rec_msg)
            
            self.is_recording_episode = True
        else:
            # Stop recording episode (exit early)
            self.get_logger().info("Stopping EPISODE recording (sending exit_early)")
            
            event_msg = String()
            event_msg.data = "exit_early"
            self._event_publisher.publish(event_msg)

            is_rec_msg = Bool()
            is_rec_msg.data = False
            self._is_recording_publisher.publish(is_rec_msg)

            self.is_recording_episode = False

    def toggle_bag_recording(self):
        if not self.is_recording_bag:
            self.start_bag_recording()
        else:
            self.stop_bag_recording()

    def start_bag_recording(self):
        timestamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
        bag_name = f"{self.bag_name_prefix}_{timestamp}"
        
        self.get_logger().info(f"Starting BAG recording: {bag_name}")

        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri=bag_name,
            storage_id='mcap'
        )
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        # Setup subscriptions for configured topics
        # We need to find the type of the topic.
        topic_names_and_types = self.get_topic_names_and_types()
        topic_map = {name: types[0] for name, types in topic_names_and_types}

        for topic in self.topics_to_record:
            if topic in topic_map:
                topic_type_name = topic_map[topic]
                self.get_logger().info(f"Subscribing to {topic} ({topic_type_name})")
                
                # Register topic
                topic_info = rosbag2_py.TopicMetadata(
                    id=0, # ID seems to be ignored/auto-managed in py writer?
                    name=topic,
                    type=topic_type_name,
                    serialization_format='cdr'
                )
                self.writer.create_topic(topic_info)
                
                # Create subscription
                msg_type = get_message(topic_type_name)
                sub = self.create_subscription(
                    msg_type,
                    topic,
                    lambda msg, t=topic: self.bag_callback(msg, t),
                    10
                )
                self.bag_subs.append(sub)
            else:
                self.get_logger().warn(f"Topic {topic} not found! Skipping.")

        self.is_recording_bag = True

    def stop_bag_recording(self):
        self.get_logger().info("Stopping BAG recording...")
        
        if self.is_recording_episode:
             self.toggle_episode_recording() # Ensure episode is closed

        # Unsubscribe
        for sub in self.bag_subs:
            self.destroy_subscription(sub)
        self.bag_subs = []

        del self.writer
        self.writer = None
        
        self.is_recording_bag = False
        self.get_logger().info("BAG recording stopped.")

    def bag_callback(self, msg, topic_name):
        if self.writer is not None:
            self.writer.write(
                topic_name,
                serialize_message(msg),
                self.get_clock().now().nanoseconds
            )

    def keyboard_listener(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while self.running and rclpy.ok():
                 # Check if there is data to be read from stdin
                if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == 'r':
                        self.toggle_episode_recording()
                    elif key == 'b':
                        self.toggle_bag_recording()
        except Exception as e:
            self.get_logger().error(f"Keyboard listener error: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    
    # Set terminal to non-canonical mode for reading single characters
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        node = LeROS2RecordUtils()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            if node.writer:
                # Attempt clear up if ctrl-c
                pass
            node.running = False
            node.destroy_node()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        rclpy.shutdown()

if __name__ == "__main__":
    main()