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
from multiprocessing import Process
from .service.ros2_manager import ros2_manager

app = create_app()

def ros2_spin_process():
    try:
        print("Starting ROS2 Executor...")
        ros2_manager.spin()
    except KeyboardInterrupt:
        print("ROS2 spin zakończony.")
    finally:
        ros2_manager.shutdown()

def signal_handler(sig, frame):
    print("App closed...")
    ros2_manager.request_stop()
    ros2_process.join(timeout=5)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

ros2_process = Process(target=ros2_spin_process)
ros2_process.start()

if __name__ == "__main__":
    try:
        socketio.run(app, debug=False, host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        print("Flask server stopped.")
    finally:
        if ros2_process.is_alive():
            print("Signaling ROS2 process to stop...")
            ros2_manager.request_stop()
            ros2_process.join(timeout=5)
            if ros2_process.is_alive():
                print("ROS2 process did not terminate, forcing shutdown...")
                os.kill(ros2_process.pid, signal.SIGKILL)
            print("ROS2 process terminated.")