#!/usr/bin/env python3

from . import create_app, socketio
import sys
import signal

app = create_app()

def signal_handler(sig, frame):
    print('App closed...')
    from .service.ros2_manager import ros2_manager
    ros2_manager.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    from .api.web.messages_api import start_ros2_subscription
    start_ros2_subscription()
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)