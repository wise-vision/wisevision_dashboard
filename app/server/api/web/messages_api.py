from flask import jsonify, Blueprint, request
from ....server.service.ros2_manager import ros2_manager
from rosidl_runtime_py.utilities import get_message
from datetime import datetime, timedelta, timezone

messages_api = Blueprint('messages_api', __name__)


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
    
# premium feature   
@messages_api.route('/topic_echo_data_base_any/<string:topic_name>', methods=['GET'])
def get_messages_any(topic_name):
    topic_type = request.args.get('type')
    if not topic_type:
        return jsonify({'error': 'Message type is required'}), 400


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

@messages_api.route('/topic_echo_data_base_any_last_week/<string:topic_name>', methods=['GET'])
def get_messages_last7days(topic_name):
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


@messages_api.route('/topic_echo/<string:topic_name>', methods=['GET'])
def echo_topic_message(topic_name):
    topic_type = request.args.get('type', 'std_msgs/msg/String')  # Default to String if not specified
    message = ros2_manager.get_topic_message(topic_name, topic_type)
    return jsonify({'message': message}), 200

@messages_api.route('/create_automatic_action', methods=['POST'])
def create_automatic_action():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400

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

    try:
        success = ros2_manager.call_delete_combined_automatic_action_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    
@messages_api.route('/available_topics', methods=['GET'])
def available_topics():
    try:
        available_topics = ros2_manager.call_available_topics_service()
        return jsonify({'available_topics': available_topics}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    
# Gps Devices API
@messages_api.route('/add_gps_device', methods=['POST'])
def add_gps_device():
    data = request.get_json()
    if not data:
        return jsonify({'error': 'Request must be JSON'}), 400

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

    try:
        success = ros2_manager.call_modify_gps_device_service(data)
        return jsonify({'success': success}), 200 if success else 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    
@messages_api.route('/message_type/<string:topic_name>', methods=['GET'])
def get_message_type(topic_name):
    try:
        message_type = ros2_manager.get_topic_message_type(topic_name)
        print('Message type:', message_type)
        return jsonify({'message_type': message_type}), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@messages_api.route('/message_structure/<path:message_type>', methods=['GET'])
def get_message_structure(message_type):
    print('Message type:', message_type)
    try:
        structure = ros2_manager.get_message_structure(message_type)
        return jsonify(structure), 200
    except Exception as e:
        return jsonify({'error': str(e)}), 500
    
def start_ros2_subscription():
    from app.server import socketio
    def push_notification_callback(notification_data):
        socketio.emit('new_notification', notification_data)
        print('New notification:', notification_data)

    ros2_manager.start_dynamic_notification_listener(push_notification_callback)

