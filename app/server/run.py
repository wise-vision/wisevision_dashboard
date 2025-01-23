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
import signal

app = create_app()

def signal_handler(sig, frame):
    print('App closed...')
    from .service.ros2_manager import ros2_manager
    ros2_manager.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)