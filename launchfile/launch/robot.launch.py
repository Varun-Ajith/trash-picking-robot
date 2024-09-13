from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    perception_node = Node(package = 'perception', 
                           executable = 'perception', 
                           namespace = 'robot_one', 
                           name = 'perception_node',
                           parameters = [
                               {
                                   "trashDetectionTopic" : "trash_detection",
                                "componentStatusService" : "component_status"
                                }
                           ])
    brain_node = Node(package = 'brain',
                      executable = 'brain',
                      namespace = 'robot_one',
                      name = 'brain_node',
                      parameters = [
                          {
                              "cameraTopic" : "camera",
                              "trashDetectionTopic" : "trash_detection",
                              "componentStatusService" : "component_status"
                          }
                      ])
    ld.add_action(perception_node)
    ld.add_action(brain_node)
    return ld