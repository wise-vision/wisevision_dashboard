from flask import Flask, jsonify, Blueprint, request
from ....server.service.ros2_manager import ros2_manager

app = Flask(__name__)
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

@messages_api.route('/topic_echo/<string:topic_name>', methods=['GET'])
def echo_topic_message(topic_name):
    topic_type = request.args.get('type', 'std_msgs/msg/String')  # Default to String if not specified
    message = ros2_manager.get_topic_message(topic_name, topic_type)
    return jsonify({'message': message}), 200

###

@app.teardown_appcontext
def shutdown_node(exception=None):
    ros2_manager.shutdown()


app.register_blueprint(messages_api)

if __name__ == '__main__':
    app.run(debug=True)
