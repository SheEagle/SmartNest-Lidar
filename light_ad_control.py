from flask import Flask, request
import socket

app = Flask(__name__)
app.config['DEBUG'] = True
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
address = ('192.168.119.130', int(5678))
client_socket.connect(address)
client_socket.send('light_ad_control'.encode())

tem_hum = "23.35,26.2"

@app.route('/get_tem_hum')
def get_tem_hum():
    global tem_hum
    return tem_hum


@app.route('/put_tem_hum', methods=['GET'])
def put_tem_hum():
    global tem_hum
    tem_hum = request.args.get("tem_hum")
    return "success"


@app.route('/light_control', methods=['GET'])
def light_control():
    command = request.args.get("command")
    client_socket.send(command.encode())
    return "success"


@app.route('/airconditioner_control', methods=['GET'])
def airconditioner_control():
    command = request.args.get("command")
    if command == "on":
        tem = request.args.get("tem")
        send_txt = command + '-' + tem
        client_socket.send(send_txt.encode())
    else:
        client_socket.send(command.encode())
    return "success"


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8888)

