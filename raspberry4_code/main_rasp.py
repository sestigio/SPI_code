import paho.mqtt.client as mqtt
import json
import os
import threading  # Necesario para hacer dos cosas a la vez

# --- Configuración ---
BROKER = "localhost"
PORT = 1883
TOPIC_TELEMETRIA = "SPI_project/telemetria"
TOPIC_COMANDOS = "SPI_project/comandos"

def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode()
        data = json.loads(payload)

        # --- NUEVO: Limpia la pantalla para que la telemetría no se desplace ---
        os.system('cls' if os.name == 'nt' else 'clear')
        
        print("\n" + "="*50)
        print(f" LOG TFG - DATOS RECIBIDOS EN TOPIC: {msg.topic}")
        print("="*50)

        # 1. ESTADOS PRINCIPALES
        print(f"[SISTEMA] Estado: {data.get('curr_st', 'N/A')} | Previo: {data.get('prior_st', 'N/A')}")
        print(f"[CONTROL] Bearing: {data.get('bearing', 'N/A')} | Media: {data.get('mean', 'N/A')}")
        print(f"[ESTADOS] Servo: {data.get('servo_st', 'N/A')} | Gyro: {data.get('gyro_st', 'N/A')}")
        
        # 2. ACELERACIONES (Global y Sensores 1, 2, 3)
        print("-" * 50)
        print(" ACELERACIONES (m/s²)")
        print(f" Global: X:{data.get('x_accel', 0):.2f} Y:{data.get('y_accel', 0):.2f} Z:{data.get('z_accel', 0):.2f}")
        print(f" Sens 1: X:{data.get('x1_accel', 0):.2f} Y:{data.get('y1_accel', 0):.2f} Z:{data.get('z1_accel', 0):.2f}")
        print(f" Sens 2: X:{data.get('x2_accel', 0):.2f} Y:{data.get('y2_accel', 0):.2f} Z:{data.get('z2_accel', 0):.2f}")
        print(f" Sens 3: X:{data.get('x3_accel', 0):.2f} Y:{data.get('y3_accel', 0):.2f} Z:{data.get('z3_accel', 0):.2f}")

        # 3. SENSORES DE POTENCIA (INA219)
        print("-" * 50)
        print(" MONITOREO ENERGÍA SERVOS")
        print(f" SERVO 1: {data.get('s1_curr', 0):.2f}mA | Shunt: {data.get('s1_v_sh', 0):.2f}V | Bus: {data.get('s1_v_bus', 0):.2f}V")
        print(f" SERVO 2: {data.get('s2_curr', 0):.2f}mA | Shunt: {data.get('s2_v_sh', 0):.2f}V | Bus: {data.get('s2_v_bus', 0):.2f}V")
        print("="*50)
        #print("\nINTRODUCE ÁNGULO (-90 a 90): ", end="", flush=True)

        # --- NUEVO: Usamos un print sin salto de línea para el prompt ---
        print("\nINTRODUCE ÁNGULO (-90 a 90): ", end="", flush=True)
        
    except Exception as e:
        print(f"Error al procesar mensaje: {e}")

# --- Configuración del cliente ---
client = mqtt.Client()
client.on_message = on_message

print(f"Conectando al broker {BROKER}...")
try:
    client.connect(BROKER, PORT, 60)
    client.subscribe(TOPIC_TELEMETRIA)
    
    # Iniciamos el bucle de red en un hilo separado (no bloqueante)
    client.loop_start() 
    print(f"Suscrito a '{TOPIC_TELEMETRIA}'.")

    # --- "API" Manual por Consola ---
    while True:
        try:
            # Quitamos el \n inicial para que no salte línea extra
            input_val = input()
            
            # Validamos que sea un número antes de enviar
            angulo = int(input_val)
            if -90 <= angulo <= 90:
                client.publish(TOPIC_COMANDOS, str(angulo))
                print(f"-> Comando enviado: {angulo}")
            else:
                print("!! Error: El ángulo debe estar entre -90 y 90.")
                
        except ValueError:
            print("!! Error: Introduce un número entero válido.")

except KeyboardInterrupt:
    print("\nCerrando sistema...")
    client.loop_stop()
    client.disconnect()