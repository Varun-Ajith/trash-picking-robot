import rclpy
from rclpy.node import Node

from custom_interfaces.srv import ComponentStatus
from std_msgs.msg import String

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class BrainNode(Node):
    def __init__(self):
        super().__init__("brain_node")

        self.trash_detection_topic_ = self.declare_parameter(
            "trashDetectionTopic",
            "None"
        )
        self.component_status_service_ = self.declare_parameter(
            "componentStatusService",
            "None"
        )

        self.get_parameter_value()
        self.callback_group_ = ReentrantCallbackGroup()

        self.trash_detection_subscriber_ = self.create_subscription(
            String,
            self.trash_detection_topic_,
            self.trash_detection_callback_,
            10,
            callback_group = self.callback_group_
        )
        self.component_status_service_timer = self.create_timer(
            5,
            self.component_status_request_,
            callback_group = self.callback_group_
        )
        self.component_status_client_ = self.create_client(
            ComponentStatus, 
            self.component_status_service_
        )

        while not self.component_status_client_.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Service not available, waiting again...")
        self.request = ComponentStatus.Request()
    
    def get_parameter_value(self):
        self.trash_detection_topic_ = self.get_parameter(
            "trashDetectionTopic").get_parameter_value().string_value
        self.component_status_service_ = self.get_parameter(
            "componentStatusService").get_parameter_value().string_value

    def trash_detection_callback_(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        message = msg.data
        if "trash" in message:
            self.get_logger().info("Sending move request to atuator")
        else:
            self.get_logger().info("Sending no request to actuator")
    async def component_status_request_(self):
        self.request.component = "camera"
        self.future = self.component_status_client_.call_async(self.request)
        result = await self.future
        self.get_logger().info(f"Service response is : {result.status}")


def main(args = None):
    rclpy.init(args = args)
    brainNode = BrainNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(brainNode, executor)
    brainNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()