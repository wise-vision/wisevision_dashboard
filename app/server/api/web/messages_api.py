from flask import Blueprint, jsonify


from ....server.service.ros_topics import get_ros2_topics
from ....server.data_object.message_objects import ROS2Topics, ROS2Topic


messages_api = Blueprint('messages_api', __name__)

@messages_api.route('/topics', methods=['GET'])  # Define the route properly
def list_topics():
    """
    Get all ROS2 visible topics
    """
    try:
        topics_obj = get_ros_topics()  # Assuming this function returns an instance of ROS2Topics
        topics_list = [{'name': topic.name, 'type': topic.type} for topic in topics_obj.get_topics()]

        return jsonify(topics_list), 200

    except Exception as e:
        return jsonify({'error': str(e)}), 500
