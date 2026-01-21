import paho.mqtt.client as mqtt
import json

# Configuración
BROKER = "localhost"  # Como el script corre en la propia RPi, usamos localhost
PORT = 1883
TOPIC = "test"

# Esta es la función "Handler" (Callback) que se ejecuta al recibir un mensaje
def on_message(client, userdata, msg):
    try:
        # Decodificamos el mensaje de bytes a string y luego a JSON
        payload = msg.payload.decode()
        data = json.loads(payload)
        
        # Imprimimos los datos de forma ordenada
        print("-" * 30)
        print(f"Estado Actual:  {data.get('curr_st', 'N/A')}")
        print(f"Estado Previo:  {data.get('prior_st', 'N/A')}")
        print(f"Servo Bearing:  {data.get('bearing', 'N/A')}")
        print(f"Media Móvil:    {data.get('mean', 'N/A')}")
        
    except Exception as e:
        print(f"Error al procesar mensaje: {e}")
        print(f"Payload original: {msg.payload}")

# Configuración del cliente
client = mqtt.Client()
client.on_message = on_message

# Conexión
print(f"Conectando al broker {BROKER}...")
client.connect(BROKER, PORT, 60)

# Suscripción al topic
client.subscribe(TOPIC)
print(f"Suscrito a {TOPIC}. Esperando datos del ESP32...")

# Bucle infinito para mantener el script escuchando
client.loop_forever()
print(f"Bearing del servo: {data['bearing']}")
