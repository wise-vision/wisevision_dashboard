#
#  Copyright (C) 2025 wisevision
#
#  SPDX-License-Identifier: MPL-2.0
#
#  This Source Code Form is subject to the terms of the Mozilla Public
#  License, v. 2.0. If a copy of the MPL was not distributed with this
#  file, You can obtain one at https://mozilla.org/MPL/2.0/.
#

from flask import Flask
from flask_socketio import SocketIO
from flask_cors import CORS


socketio = SocketIO(cors_allowed_origins="*")
app = Flask(__name__)


def create_app():
    app = Flask(__name__)
    CORS(app)

    from .api.web.messages_api import messages_api
    app.register_blueprint(messages_api, url_prefix='/api')

    socketio.init_app(app)

    return app