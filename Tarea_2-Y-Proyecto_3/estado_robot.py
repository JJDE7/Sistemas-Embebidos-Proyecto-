import paho.mqtt.client as mqtt
import json
from rich import print
from rich.console import Console
from rich.table import Table
from datetime import datetime

console = Console()

def on_connect(client, userdata, flags, rc):
    client.subscribe("robot/estado")
    print("[MONITOR] Suscrito a robot/estado")

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        table = Table(title=f"[bold blue]Estado del Robot - {timestamp}[/bold blue]")
        table.add_column("Campo", style="cyan", no_wrap=True)
        table.add_column("Valor", style="magenta")
        for key, value in data.items():
            table.add_row(str(key), str(value))
        console.print(table)
        # Logging en archivo opcional:
        with open("registro_estado.txt", "a") as f:
            f.write(f"{timestamp} - {json.dumps(data)}\n")
    except Exception as e:
        print(f"[ERROR] Mensaje malformado: {e}")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("192.168.1.10", 1883)
print("[MONITOR] Esperando estados de robots...")
client.loop_forever()
