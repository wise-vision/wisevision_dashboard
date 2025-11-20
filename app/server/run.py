#!/usr/bin/env python3
#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#


from . import create_app, socketio
import sys
import os
import signal
from threading import Thread
from .service.ros2_manager import ros2_manager

app = create_app()

def ros2_spin_thread():
    try:
        print("Starting ROS2 Executor in thread...")
        ros2_manager.spin()
    except KeyboardInterrupt:
        print("ROS2 spin zakończony.")
    finally:
        ros2_manager.shutdown()

def signal_handler(sig, frame):
    print("App closed...")
    ros2_manager.request_stop()
    ros2_thread.join(timeout=5)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

ros2_thread = Thread(target=ros2_spin_thread, daemon=True)
ros2_thread.start()

if __name__ == "__main__":
    try:
        socketio.run(app, debug=False, host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        print("Flask server stopped.")
    finally:
        if ros2_thread.is_alive():
            print("Signaling ROS2 thread to stop...")
            ros2_manager.request_stop()
            ros2_thread.join(timeout=5)
            print("ROS2 thread terminated.")