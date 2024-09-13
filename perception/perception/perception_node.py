import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from custom_interfaces.srv import ComponentStatus


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")

        self.camera_topic_ = self.declare_parameter(
            "cameraTopic",
            "None"
        )

        self.trash_detection_topic_ = self.declare_parameter(
            "trashDetectionTopic",
            "None"
        )

        self.componentStatusService = self.declare_parameter(
            "componentStatusService",
            "None"
        )

        self.get_parameter_values()
        
        self.camerSubscriber_ = self.create_subscription(
            String,
            self.camera_topic_,
            self.camera_callback_,
            10
        )
        self.trashDetectionPublisher_ = self.create_publisher(
            String,
            self.trash_detection_topic_,
            10
        )
        self.componentStatusService = self.create_service(
            ComponentStatus,
            self.componentStatusService,
            self.handle_component_status_
        )
    def get_parameter_values(self):
        self.camera_topic_ = self.get_parameter(
            "cameraTopic" ).get_parameter_value().string_value
        self.trash_detection_topic_ = self.get_parameter(
            "trashDetectionTopic").get_parameter_value().string_value
        self.componentStatusService = self.get_parameter(
            "componentStatusService").get_parameter_value().string_value


    def camera_callback_(self, message):
        msg = String()
        if(message.data == "trash"):
            msg.data = "I see trash"
        else:
            msg.data = "I dont see trash"
        self.trashDetectionPublisher_.publish(msg)
        self.get_logger().info("Publshing : %s" % msg.data)
    
    def handle_component_status_(self, request, response):
        self.get_logger().info("Server called!")
        if(request.component == "camera"):
            response.status = "Camera battery is at 100%"
        else:
            response.status = "Component unavailable"
        return response
def main(args = None):
    rclpy.init(args = args)
    perceptionNode = PerceptionNode()
    rclpy.spin(perceptionNode)
    perceptionNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()