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