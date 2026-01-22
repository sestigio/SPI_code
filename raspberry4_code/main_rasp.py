from flask import Flask, render_template
from flask_socketio import SocketIO
import paho.mqtt.client as mqtt
import json

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

# --- Configuración MQTT ---
BROKER = "localhost"
TOPIC_TELEMETRIA = "SPI_project/telemetria"
TOPIC_COMANDOS = "SPI_project/comandos"

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        # Enviamos los datos recibidos por MQTT directamente a la página web
        socketio.emit('mqtt_data', data)
    except:
        pass

mqtt_client = mqtt.Client()
mqtt_client.on_message = on_message
mqtt_client.connect(BROKER, 1883, 60)
mqtt_client.subscribe(TOPIC_TELEMETRIA)
mqtt_client.loop_start()

@app.route('/')
def index():
    return render_template('index.html')

# Cuando mueves el slider en la web, se activa esta función
@socketio.on('enviar_comando')
def handle_comando(valor):
    mqtt_client.publish(TOPIC_COMANDOS, str(valor))
    print(f"Comando enviado al ESP32: {valor}")

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)