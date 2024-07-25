from flask import Flask

def create_app():
    app = Flask(__name__)

    from .api.web.messages_api import messages_api
    app.register_blueprint(messages_api, url_prefix='/api')

    return app
