#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#

from flask import jsonify, Blueprint, request
from ....server.service.ros2_manager import ros2_manager
from rosidl_runtime_py.utilities import get_message
from datetime import datetime, timedelta, timezone
from urllib.parse import unquote
import re

messages_api = Blueprint('messages_api', __name__)


def is_valid_ros2_topic_name(topic_name):
    """
    Validates a topic_name against ROS 2 naming conventions.

    - Can start with a `/`
    - Cannot end with a `/`
    - Cannot contain `//`
    - Can only include alphanumeric characters, `_`, `/`
    - Cannot be a numeric string
    """

    if not topic_name:
        return False

    if topic_name.isdigit():
        return False

    pattern = r'^(?!.*//)[a-zA-Z0-9_/]+(?<!/)$'
    if re.match(pattern, topic_name):
        return True

    return False

def is_valid_ros2_topic_name_struct(topic_name):
    """
    Validates a topic_name against ROS 2 naming conventions for _struct topics.

    - Can start with `%`
    - Can contain alphanumeric characters, `_`, `/`, `%`
    - Cannot contain `//`
    - Cannot contain consecutive `%` (e.g., `%%`)
    - Cannot end with `/`
    - Must include '_struct' as a suffix
    - Cannot be a numeric string
    """

    if not topic_name or topic_name.isdigit():
        return False

    pattern = r'^(%?[a-zA-Z0-9_/%.]+)(?<!/)$'
    if not re.match(pattern, topic_name) or '//' in topic_name or '%%' in topic_name:
        return False

    return True

def is_valid_message_type(message_type):
    """
    Validates if the `listen_message_type` is correct:
    - Contains exactly two `/`.
    - The middle part (after the first `/`) must be `msg` or `srv`.
    """
    if not message_type:
        return False

    parts = message_type.split('/')

    if len(parts) != 3:
        return False

    if parts[1] not in ['msg', 'srv']:
        return False

    return True

def is_valid_message_type_black_box(message_type):
    """
    Validates if the `message_type` is correct in a black-box manner:
    - Contains exactly one `/`.
    - Does not validate the middle part of the string.
    """
    if not message_type:
        return False

    parts = message_type.split('/')

    if len(parts) != 2:
        return False

    return True

def extract_topics_from_expression(expression):
    """
    Extract topics from a logic_expression string.
    Example:
    - Input: "(topic1 and topic2) or topic3"
    - Output: ["topic1", "topic2", "topic3"]
    """
    raw_topics = re.findall(r'\b[a-zA-Z0-9_/]+\b', expression)
    logical_operators = {"and", "or"}
    return list(set(topic for topic in raw_topics if topic.lower() not in logical_operators))

@messages_api.route('/topics', methods=['GET'])
def list_topics():
    try:
        default_filter = request.args.get('default_filter', 'true').lower() == 'true'
        message_types = request.args.getlist('message_types')
        message_namespaces = request.args.getlist('message_namespaces')
        name_contains = request.args.getlist('name_contains')

        message_types = message_types if message_types else None
        message_namespaces = message_namespaces if message_namespaces else None
        name_contains = name_contains if name_contains else None

        topics = ros2_manager.get_topic_list(
            default_filter=default_filter, 
            message_types=message_types, 
            message_namespaces=message_namespaces,
            name_contains=name_contains
        )

        topics_list = [{'name': name, 'type': topic_type} for name, topic_type in topics]
        return jsonify(topics_list), 200

    except Exception as e:
        return jsonify({'error': str(e)}), 500
    
@messages_api.route('/topic_types', methods=['GET'])
def list_topic_types():
    try:
        topic_types = ros2_manager.get_topics_types()
        return jsonify(topic_types), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    
@messages_api.route('/namespaces', methods=['GET'])
def list_namespaces():
    try:
        namespaces = ros2_manager.get_namespaces()
        return jsonify(namespaces), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@messages_api.route('/services', methods=['GET'])
def list_services():
    services_obj = ros2_manager.get_service_list()
    services_list = [{'name': service.name, 'type': service.type} for service in services_obj.get_services()]
    return jsonify(services_list), 200

@messages_api.route('/start_record_topics', methods=['POST'])
def start_record_topics():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400
    try:
        success = ros2_manager.call_start_record_topics_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@messages_api.route('/create_bucket', methods=['POST'])
def create_bucket():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400
    try:
        success = ros2_manager.call_create_bucket_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@messages_api.route('/get_influx_buckets', methods=['GET'])
def get_influx_buckets():
    try:
        resp = ros2_manager.call_get_influx_buckets_service()

        influx_buckets = list(getattr(resp, 'influx_buckets', [])) if resp else []
        error_message  = getattr(resp, 'error_message', '')

        return jsonify({
            'success': bool(influx_buckets),
            'influx_buckets': influx_buckets,
            'error_message': error_message
        }), 200

    except TimeoutError as e:
        return jsonify({'error': str(e)}), 503
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@messages_api.route('/stop_record_topics', methods=['POST'])
def stop_record_topics():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400
    try:
        success = ros2_manager.call_stop_record_topics_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@messages_api.route('/get_recorded_bags_by_topic', methods=['POST'])
def get_recorded_bags_by_topic():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400

    try:
        resp = ros2_manager.call_get_recorded_bags_by_topic_service(data)  # <-- dict
        status = 200 if resp.get("success") else 400
        return jsonify(resp), status
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@messages_api.route('/get_messages', methods=['POST'])
def get_messages():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400
    try:
        response = ros2_manager.call_get_messages_service(data)
        return jsonify({
            'success': response.success,
            'messages': list(response.messages),
            'timestamps': [str(ts) for ts in response.timestamps],
            'message_type': response.message_type,
            'error_message': response.error_message
        }), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@messages_api.route('/get_recorded_topics', methods=['GET'])
def get_recorded_topics():
    try:
        response = ros2_manager.call_get_recorded_topics_service()
        return jsonify({
            'success': response.success,
            'topics': list(response.topics),
            'bucket_names': list(response.bucket_names),
            'error_message': response.error_message
        }), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@messages_api.route('/get_currently_recording_topics', methods=['GET'])
def get_currently_recording_topics():
    try:
        response = ros2_manager.call_get_currently_recording_topics_service()
        return jsonify({
            'success': response.success,
            'topics': list(response.topics),
            'bucket_names': list(response.bucket_names),
            'record_ids': list(response.record_ids),
            'error_message': response.error_message
        }), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@messages_api.route('/play_recordings', methods=['POST'])
def play_recordings():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400
    try:
        success = ros2_manager.call_play_recordings_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@messages_api.route('/stop_playing_topics', methods=['POST'])
def stop_playing_topics():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400
    try:
        success = ros2_manager.call_stop_playing_topics_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@messages_api.route('/get_currently_playing_topics', methods=['GET'])
def get_currently_playing_topics():
    try:
        response = ros2_manager.call_get_currently_playing_topics_service()
        return jsonify({
            'success': response.success,
            'topics': list(response.topics),
            'bucket_names': list(response.bucket_names),
            'record_ids': list(response.record_ids),
            'error_message': response.error_message
        }), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    

@messages_api.route('/get_pending_recording_topics', methods=['GET'])
def get_pending_recording_topics():
    try:
        response = ros2_manager.call_get_pending_recording_topics_service()
        return jsonify({
            'success': response.success,
            'topics': list(response.topics),
            'error_message': response.error_message
        }), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@messages_api.route('/topic_echo/<path:topic_name>', methods=['GET'])
def echo_topic_message(topic_name):
    topic_name = unquote(topic_name)
    if not topic_name:
        return jsonify({'error': 'topic_name is required.'}), 400
    if not is_valid_ros2_topic_name(topic_name):
        return jsonify({'error': f"Invalid topic_name '{topic_name}'. Please follow ROS 2 naming conventions."}), 400
    topic_type = request.args.get('type', 'std_msgs/msg/String')  # Default to String if not specified
    message = ros2_manager.get_topic_message(topic_name, topic_type)
    return jsonify({'message': message}), 200

@messages_api.route('/create_automatic_action', methods=['POST'])
def create_automatic_action():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400
    
    listen_topic = data.get('listen_topic', None)
    if not listen_topic:
        return jsonify({'error': 'topic_name is required.'}), 400
    if not is_valid_ros2_topic_name(listen_topic):
        return jsonify({'error': f"Invalid topic_name '{listen_topic}'. Please follow ROS 2 naming conventions."}), 400
    action_and_publisher_name = data.get('action_and_publisher_name', None)
    if not action_and_publisher_name:
        return jsonify({'error': 'topic_name is required.'}), 400
    if not is_valid_ros2_topic_name(action_and_publisher_name):
        return jsonify({'error': f"Invalid topic_name '{action_and_publisher_name}'. Please follow ROS 2 naming conventions."}), 400
    listen_message_type = data['listen_message_type']
    if not is_valid_message_type(listen_message_type):
        return jsonify({'error': f"Invalid listen_message_type '{listen_message_type}'. It must follow the format <package>/msg_or_srv/<type>."}), 400
    pub_msg_type = data.get('pub_msg_type', 'std_msgs/msg/String')
    if not is_valid_message_type(pub_msg_type):
        return jsonify({'error': f"Invalid pub_msg_type '{pub_msg_type}'. It must follow the format <package>/msg_or_srv/<type>."}), 400


    try:
        success = ros2_manager.call_automatic_action_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    
@messages_api.route('/delete_automatic_action', methods=['POST'])
def delete_automatic_action():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400
    listen_topic_to_delete = data.get('listen_topic_to_delete', None)
    if not listen_topic_to_delete:
        return jsonify({'error': 'listen_topic_to_delete is required.'}), 400
    if not is_valid_ros2_topic_name(listen_topic_to_delete):
        return jsonify({'error': f"Invalid listen_topic_to_delete '{listen_topic_to_delete}'. Please follow ROS 2 naming conventions."}), 400

    try:
        success = ros2_manager.call_delete_automatic_action_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@messages_api.route('/create_combined_automatic_action', methods=['POST'])
def create_combined_automatic_action():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400
    if not isinstance(data, dict):
        return jsonify({'error': 'Invalid data format. JSON object expected.'}), 400

    listen_topics = data.get('listen_topics', [])
    if not isinstance(listen_topics, list) or not listen_topics:
        return jsonify({'error': 'listen_topic must be a non-empty list.'}), 400

    invalid_topics = [topic for topic in listen_topics if not is_valid_ros2_topic_name(topic)]
    if invalid_topics:
        return jsonify({'error': f"Invalid topics in listen_topic: {invalid_topics}"}), 400

    logic_expression = data.get('logic_expression', '')
    if not logic_expression or not isinstance(logic_expression, str):
        return jsonify({'error': 'logic_expression must be a non-empty string.'}), 400

    num_topics = len(listen_topics)
    operators = re.findall(r'\b(and|or)\b', logic_expression)
    num_operators = len(operators)
    if num_operators != num_topics - 1:
        return jsonify({
            "error": "Invalid logic_expression",
            "message": f"Expected {num_topics - 1} operators ('and' or 'or'), but found {num_operators}."
        }), 400
    
    missing_topics = [topic for topic in extract_topics_from_expression(logic_expression) if topic not in listen_topics]
    if missing_topics:
        return jsonify({'error': f"logic_expression contains topics not in listen_topic: {missing_topics}"}), 400

    action_and_publisher_name = data.get('action_and_publisher_name', '')
    if not action_and_publisher_name or not is_valid_ros2_topic_name(action_and_publisher_name):
        return jsonify({'error': f"Invalid action_and_publisher_name: {action_and_publisher_name}"}), 400

    try:
        success = ros2_manager.call_combined_automatic_action_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@messages_api.route('/delete_combined_automatic_action', methods=['POST'])
def delete_combined_automatic_action():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400
    name_of_combined_topics_publisher = data.get('name_of_combined_topics_publisher', '')
    if not name_of_combined_topics_publisher or not is_valid_ros2_topic_name(name_of_combined_topics_publisher):
        return jsonify({'error': f"Invalid listen_topic_to_delete: {name_of_combined_topics_publisher}"}), 400

    try:
        success = ros2_manager.call_delete_combined_automatic_action_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    
@messages_api.route('/available_topics', methods=['GET'])
def available_topics():
    try:
        available_topics = ros2_manager.call_available_topics_service()
        return jsonify({'available_topics_with_parameters_and_time': available_topics}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    
@messages_api.route('/available_topics_combined', methods=['GET'])
def available_topics_combined():
    try:
        available_topics = ros2_manager.call_available_topics_combined_service()
        return jsonify({'available_combined_topics_with_parameters_and_time': available_topics}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    
@messages_api.route('/change_automatic_action', methods=['POST'])
def change_automatic_action():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400
    if not is_valid_ros2_topic_name(data['action_and_publisher_name_to_change']):
        return jsonify({'error': f"Invalid action_and_publisher_name_to_change: {data['action_and_publisher_name_to_change']}"}), 400
    if not is_valid_ros2_topic_name(data['listen_topic']):
        return jsonify({'error': f"Invalid listen_topic: {data['listen_topic']}"}), 400
    if not is_valid_ros2_topic_name(data['new_action_and_publisher_name']):
        return jsonify({'error': f"Invalid new_action_and_publisher_name: {data['new_action_and_publisher_name']}"}), 400

    if not is_valid_message_type(data['listen_message_type']):
        return jsonify({'error': f"Invalid listen_message_type: {data['listen_message_type']}"}), 400
    if not is_valid_message_type(data['pub_message_type']):
        return jsonify({'error': f"Invalid pub_message_type: {data['pub_message_type']}"}), 400

    try:
        success = ros2_manager.call_change_automatic_action_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    
@messages_api.route('/change_automatic_action_combined', methods=['POST'])
def change_automatic_action_combined():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400
    if not is_valid_ros2_topic_name(data['action_and_publisher_name_to_change']):
        return jsonify({'error': f"Invalid action_and_publisher_name_to_change: {data['action_and_publisher_name_to_change']}"}), 400

    listen_topics = data.get('listen_topics', [])
    if not isinstance(listen_topics, list) or not listen_topics:
        return jsonify({'error': 'listen_topics must be a non-empty list.'}), 400

    invalid_topics = [topic for topic in listen_topics if not is_valid_ros2_topic_name(topic)]
    if invalid_topics:
        return jsonify({'error': f"Invalid topics in listen_topics: {invalid_topics}"}), 400

    logic_expression = data.get('logic_expression', '')
    if not logic_expression or not isinstance(logic_expression, str):
        return jsonify({'error': 'logic_expression must be a non-empty string.'}), 400
    
    num_topics = len(listen_topics)
    operators = re.findall(r'\b(and|or)\b', logic_expression)
    num_operators = len(operators)
    if num_operators != num_topics - 1:
        return jsonify({
            "error": "Invalid logic_expression",
            "message": f"Expected {num_topics - 1} operators ('and' or 'or'), but found {num_operators}."
        }), 400

    missing_topics = [topic for topic in extract_topics_from_expression(logic_expression) if topic not in listen_topics]
    if missing_topics:
        return jsonify({'error': f"logic_expression contains topics not in listen_topics: {missing_topics}"}), 400

    if not is_valid_ros2_topic_name(data['new_action_and_publisher_name']):
        return jsonify({'error': f"Invalid new_action_and_publisher_name: {data['new_action_and_publisher_name']}"}), 400

    try:
        success = ros2_manager.call_change_automatic_action_combined_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    
# Gps Devices API
@messages_api.route('/topic_echo_gps_devices', methods=['GET'])
def echo_topic_gps_devices():
    message = ros2_manager.get_gps_devices_message()
    return jsonify({'message': message}), 200

@messages_api.route('/add_gps_device', methods=['POST'])
def add_gps_device():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400
    if "device_name" not in data or not data["device_name"]:
            return jsonify({"error": "Missing or invalid 'device_name'"}), 400

    if "device_eui" not in data or not isinstance(data["device_eui"], dict):
        return jsonify({"error": "Missing or invalid 'device_eui'"}), 400

    if "data" not in data["device_eui"] or not isinstance(data["device_eui"]["data"], list) or len(data["device_eui"]["data"]) != 8:
        return jsonify({"error": "Invalid 'device_eui.data', must be a list of 8 integers"}), 400

    if any(not isinstance(e, int) or e < 0 or e > 255 for e in data["device_eui"]["data"]):
        return jsonify({"error": "'device_eui.data' must contain 8 integers between 0 and 255"}), 400

    if "nav_value" not in data or not isinstance(data["nav_value"], dict):
        return jsonify({"error": "Missing or invalid 'nav_value'"}), 400

    if not all(k in data["nav_value"] for k in ["latitude", "longitude", "altitude"]):
        return jsonify({"error": "'nav_value' must contain 'latitude', 'longitude', and 'altitude'"}), 400

    if not all(isinstance(data["nav_value"].get(k), (float, int)) for k in ["latitude", "longitude", "altitude"]):
        return jsonify({"error": "'nav_value' must contain numeric 'latitude', 'longitude', and 'altitude'"}), 400

    if "is_moving" not in data or not isinstance(data["is_moving"], bool):
        return jsonify({"error": "Missing or invalid 'is_moving'"}), 400

    try:
        success = ros2_manager.call_add_gps_device_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    
@messages_api.route('/delete_gps_device', methods=['POST'])
def delete_gps_device():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400

    try:
        success = ros2_manager.call_delete_gps_device_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@messages_api.route('/modify_gps_device', methods=['POST'])
def modify_gps_device():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400
    if "device_name" not in data or not data["device_name"]:
            return jsonify({"error": "Missing or invalid 'device_name'"}), 400

    if "device_eui" not in data or not isinstance(data["device_eui"], dict):
        return jsonify({"error": "Missing or invalid 'device_eui'"}), 400

    if "data" not in data["device_eui"] or not isinstance(data["device_eui"]["data"], list) or len(data["device_eui"]["data"]) != 8:
        return jsonify({"error": "Invalid 'device_eui.data', must be a list of 8 integers"}), 400

    if any(not isinstance(e, int) or e < 0 or e > 255 for e in data["device_eui"]["data"]):
        return jsonify({"error": "'device_eui.data' must contain 8 integers between 0 and 255"}), 400

    if "nav_value" not in data or not isinstance(data["nav_value"], dict):
        return jsonify({"error": "Missing or invalid 'nav_value'"}), 400

    if not all(k in data["nav_value"] for k in ["latitude", "longitude", "altitude"]):
        return jsonify({"error": "'nav_value' must contain 'latitude', 'longitude', and 'altitude'"}), 400

    if not all(isinstance(data["nav_value"].get(k), (float, int)) for k in ["latitude", "longitude", "altitude"]):
        return jsonify({"error": "'nav_value' must contain numeric 'latitude', 'longitude', and 'altitude'"}), 400

    try:
        success = ros2_manager.call_modify_gps_device_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    
@messages_api.route('/message_type/<string:topic_name>', methods=['GET'])
def get_message_type(topic_name):
    if not topic_name:
        return jsonify({'error': 'topic_name is required.'}), 400
    if not is_valid_ros2_topic_name_struct(topic_name):
        return jsonify({'error': f"Invalid topic_name '{topic_name}'. Please follow ROS 2 naming conventions."}), 400
    try:
        message_type = ros2_manager.get_topic_message_type(topic_name)
        return jsonify({'message_type': message_type}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@messages_api.route('/message_structure/<path:message_type>', methods=['GET'])
def get_message_structure(message_type):
    try:   
        structure = ros2_manager.get_message_structure(message_type)
        include_types = request.args.getlist('include_types')
        exclude_types = request.args.getlist('exclude_types')
        if include_types or exclude_types:
            structure = ros2_manager.filter_message_structure(structure, include_types, exclude_types) 
        return jsonify(structure), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    
@messages_api.route('/message_field_types/<path:message_type>', methods=['GET'])
def get_message_field_types(message_type):
    try:
        structure = ros2_manager.get_message_structure(message_type)  # Retrieve the full structure
        field_types = ros2_manager.get_message_field_types(structure)  # Extract unique field types

        return jsonify(field_types), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    

