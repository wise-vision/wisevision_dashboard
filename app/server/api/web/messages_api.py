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
        topics_obj = ros2_manager.get_topic_list()
        topics_list = [{'name': topic.name, 'type': topic.type}
                       for topic in topics_obj.get_topics()]
        return jsonify(topics_list), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@messages_api.route('/services', methods=['GET'])
def list_services():
    services_obj = ros2_manager.get_service_list()
    services_list = [{'name': service.name, 'type': service.type} for service in services_obj.get_services()]
    return jsonify(services_list), 200

@messages_api.route('/get_messages', methods=['POST'])
def get_messages():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400

    if not any(key in data for key in ['time_start', 'time_end', 'number_of_msgs']):
        return jsonify({'error': 'Request must contain at least time_start and time_end, or number_of_msgs, or all three.'}), 400
    
    topic_name = data.get('topic_name', None)
    if not topic_name:
        return jsonify({'error': 'topic_name is required.'}), 400
    if not is_valid_ros2_topic_name(topic_name):
        return jsonify({'error': f"Invalid topic_name '{topic_name}'. Please follow ROS 2 naming conventions."}), 400

    try:
        FullDateTime = get_message('lora_msgs/msg/FullDateTime')

        if 'time_start' in data and 'time_end' in data:
            time_start = data['time_start']
            time_end = data['time_end']

            time_start_obj = FullDateTime(
                year=int(time_start.get('year', 0)),
                month=int(time_start.get('month', 0)),
                day=int(time_start.get('day', 0)),
                hour=int(time_start.get('hour', 0)),
                minute=int(time_start.get('minute', 0)),
                second=int(time_start.get('second', 0)),
                nanosecond=int(time_start.get('nanosecond', 0))
            )

            time_end_obj = FullDateTime(
                year=int(time_end.get('year', 0)),
                month=int(time_end.get('month', 0)),
                day=int(time_end.get('day', 0)),
                hour=int(time_end.get('hour', 0)),
                minute=int(time_end.get('minute', 0)),
                second=int(time_end.get('second', 0)),
                nanosecond=int(time_end.get('nanosecond', 0))
            )

            data['time_start'] = time_start_obj
            data['time_end'] = time_end_obj

        elif 'time_start' in data or 'time_end' in data:
            return jsonify({'error': 'Both time_start and time_end are required together.'}), 400

        response = ros2_manager.call_get_messages_service(data)

        if response:
            int32_msgs = list(response['int32_msgs']) if isinstance(response, dict) else list(response.int32_msgs)

            micro_publisher_data_serialized = []
            micro_publisher_data = response['micro_publisher_data'] if isinstance(response, dict) else response.micro_publisher_data
            for mp_data in micro_publisher_data:
                if isinstance(mp_data, dict):
                    sensors_data_serialized = mp_data['sensors_data']
                else: 
                    sensors_data_serialized = []
                    for sensor in mp_data.sensors_data:
                        sensors_data_serialized.append({
                            'id': sensor.id,
                            'tpb_value': {
                                'value_type': sensor.tpb_value.value_type,
                                'temperature': sensor.tpb_value.temperature,
                                'pressure': sensor.tpb_value.pressure,
                                'binary_value': sensor.tpb_value.binary_value
                            }
                        })
                micro_publisher_data_serialized.append({'sensors_data': sensors_data_serialized})

            timestamps_serialized = []
            timestamps = response['timestamps'] if isinstance(response, dict) else response.timestamps
            for ts in timestamps:
                if isinstance(ts, dict):
                    timestamps_serialized.append(ts)
                else:
                    timestamps_serialized.append({
                        'year': ts.year,
                        'month': ts.month,
                        'day': ts.day,
                        'hour': ts.hour,
                        'minute': ts.minute,
                        'second': ts.second,
                        'nanosecond': ts.nanosecond
                    })

            return jsonify({
                'int32_msgs': int32_msgs,
                'micro_publisher_data': micro_publisher_data_serialized,
                'timestamps': timestamps_serialized
            }), 200
        else:
            return jsonify({'error': 'No messages received from service'}), 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@messages_api.route('/topic_echo_data_base/<string:topic_name>', methods=['GET'])
def get_last_message_from_topic(topic_name):
    message_type = request.args.get('type')
    if not message_type:
        return jsonify({'error': 'Type parameter is required'}), 400
    if not is_valid_message_type_black_box(message_type):
        return jsonify({'error': f"Invalid message type '{message_type}'. It must follow the format <package>/<type>."}), 400
    if not topic_name:
        return jsonify({'error': 'topic_name is required.'}), 400
    if not is_valid_ros2_topic_name(topic_name):
        return jsonify({'error': f"Invalid topic_name '{topic_name}'. Please follow ROS 2 naming conventions."}), 400

    try:

        request_data = {
            'topic_name': topic_name,
            'message_type': message_type
        }

        response = ros2_manager.call_get_last_message_service(request_data)

        if response:
            int32_msgs = list(response['int32_msgs']) if isinstance(response, dict) else list(response.int32_msgs)

            micro_publisher_data_serialized = []
            micro_publisher_data = response['micro_publisher_data'] if isinstance(response, dict) else response.micro_publisher_data
            for mp_data in micro_publisher_data:
                if isinstance(mp_data, dict):
                    sensors_data_serialized = mp_data['sensors_data']
                else:
                    sensors_data_serialized = []
                    for sensor in mp_data.sensors_data:
                        sensors_data_serialized.append({
                            'id': sensor.id,
                            'tpb_value': {
                                'value_type': sensor.tpb_value.value_type,
                                'temperature': sensor.tpb_value.temperature,
                                'pressure': sensor.tpb_value.pressure,
                                'binary_value': sensor.tpb_value.binary_value
                            }
                        })
                micro_publisher_data_serialized.append({'sensors_data': sensors_data_serialized})

            timestamps_serialized = []
            timestamps = response['timestamps'] if isinstance(response, dict) else response.timestamps
            for ts in timestamps:
                if isinstance(ts, dict):
                    timestamps_serialized.append(ts)
                else:
                    timestamps_serialized.append({
                        'year': ts.year,
                        'month': ts.month,
                        'day': ts.day,
                        'hour': ts.hour,
                        'minute': ts.minute,
                        'second': ts.second,
                        'nanosecond': ts.nanosecond
                    })

            return jsonify({
                'int32_msgs': int32_msgs,
                'micro_publisher_data': micro_publisher_data_serialized,
                'timestamps': timestamps_serialized
            }), 200
        else:
            return jsonify({'error': 'No messages received from service'}), 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    

@messages_api.route('/add_storage_to_database', methods=['POST'])
def add_storage_to_database():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400

    try:
        success = ros2_manager.call_add_storage_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@messages_api.route('/create_database', methods=['POST'])
def create_database():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400

    try:
        success = ros2_manager.call_create_database_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500    
     
@messages_api.route('/topic_echo_data_base_any/<path:topic_name>', methods=['GET'])
def get_messages_any(topic_name):
    topic_name = unquote(topic_name)
    topic_type = request.args.get('type')
    if not topic_type:
        return jsonify({'error': 'Message type is required'}), 400
    if not is_valid_message_type(topic_type):
        return jsonify({'error': f"Invalid message type '{topic_type}'. It must follow the format <package>/msg_or_srv/<type>."}), 400
    if not topic_name:
        return jsonify({'error': 'topic_name is required.'}), 400
    if not is_valid_ros2_topic_name(topic_name):
        return jsonify({'error': f"Invalid topic_name '{topic_name}'. Please follow ROS 2 naming conventions."}), 400
    
    number_of_msgs = int(request.args.get('number_of_msgs', 0))  # Default to 0 if not specified, if 0 then all messages will be returned

    time_start = request.args.get('time_start')
    time_end = request.args.get('time_end') 

    params = {
        'topic_name': topic_name,
        'message_type': topic_type,
        'number_of_msgs': number_of_msgs
    }

    if time_start:
        params['time_start'] = time_start
    if time_end:
        params['time_end'] = time_end

    try:
        response = ros2_manager.call_get_messages_service_any(params)
        if response:
            return jsonify(response), 200
        else:
            return jsonify({'error': 'No messages found'}), 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@messages_api.route('/topic_echo_data_base_any_last_week/<path:topic_name>', methods=['GET'])
def get_messages_last7days(topic_name):
    topic_name = unquote(topic_name)
    if not topic_name:
        return jsonify({'error': 'topic_name is required.'}), 400
    if not is_valid_ros2_topic_name(topic_name):
        return jsonify({'error': f"Invalid topic_name '{topic_name}'. Please follow ROS 2 naming conventions."}), 400
    
    topic_type = request.args.get('type')
    if not topic_type:
        return jsonify({'error': 'Message type is required'}), 400

    number_of_msgs = 0  # Default to 0 to retrieve all messages

    # Use UTC datetime for the last 7 days
    today = datetime.now(timezone.utc)
    seven_days_ago = today - timedelta(days=7)

    # Convert the dates to ISO 8601 format (with timezone info)
    time_start = seven_days_ago.isoformat() 
    time_end = today.isoformat()

    params = {
        'topic_name': topic_name,
        'message_type': topic_type,
        'number_of_msgs': number_of_msgs,
        'time_start': time_start,
        'time_end': time_end
    }

    try:
        # Call to service to retrieve messages
        response = ros2_manager.call_get_messages_service_any(params)
        if response:
            return jsonify(response), 200
        else:
            return jsonify({'error': 'No messages found'}), 500
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
        return jsonify(structure), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    

