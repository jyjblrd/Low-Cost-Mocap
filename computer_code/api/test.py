from flask import Flask, render_template
from flask_socketio import SocketIO

app = Flask(__name__)
socketio = SocketIO(app, path="/api/socket.io")

@app.route("/")
def test():
    return '<script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js" integrity="sha512-q/dWJ3kcmjBLU4Qc47E4A9kTB4m3wuTY7vkFJDTZKjTs8jhyGQnaUrxa0Ytd0ssMZhbNua9hE+E7Qv1j+DyZwA==" crossorigin="anonymous"></script><script type="text/javascript" charset="utf-8">var socket = io(\'http://localhost:3001\', {path: \'/api/socket.io/\'});socket.on(\'connect\', function() {socket.emit(\'my event\', {data: \'I\m connected!\'});});</script>', 200

if __name__ == '__main__':
    socketio.run(app, port=3001, debug=True)