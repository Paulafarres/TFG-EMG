import asyncio
import threading
from bleak import BleakClient, BleakScanner
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# UUIDs del servei/característica BLE
SERVICE_UUID = "0af56af5-502d-4244-ae1d-262a9d72945d"
CHARACTERISTIC_UUID = "4fc085b6-f4a6-4142-ac13-ced0cf3ad0e3"

MAX_POINTS = 1000  

v1_vals = []
v2_vals = []
v3_vals = []

# Configuració gràfica
fig, ax = plt.subplots()
line1, = ax.plot([], [], label="Canal 1")
line2, = ax.plot([], [], label="Canal 2")
line3, = ax.plot([], [], label="Canal 3")
def init():
    ax.set_xlim(0, MAX_POINTS)
    ax.set_ylim(-100, 100)  # ajusta segons l’amplitud RMS real
    ax.set_title("EMG")
    ax.set_xlabel("Mostres")
    ax.set_ylabel("Amplitud (mV)")
    ax.legend()
    return line1,  line2, line3


def update_plot(frame):
    line1.set_data(range(len(v1_vals)), v1_vals)
    line2.set_data(range(len(v2_vals)), v2_vals)
    line3.set_data(range(len(v3_vals)), v3_vals)
    return line1, line2, line3

def handle_notification(sender, data):
    try:
        text = data.decode().strip()
        linies = text.split("\n")

        if len(linies) >= 3:
            ch1_vals = list(map(float, linies[0].split(",")))
            ch2_vals = list(map(float, linies[1].split(",")))
            ch3_vals = list(map(float, linies[2].split(",")))

            v1_vals.extend(ch1_vals)
            v2_vals.extend(ch2_vals)
            v3_vals.extend(ch3_vals)

            # Manté només els últims MAX_POINTS
            if len(v1_vals) > MAX_POINTS:
                v1_vals[:] = v1_vals[-MAX_POINTS:]
                v2_vals[:] = v2_vals[-MAX_POINTS:]
                v3_vals[:] = v3_vals[-MAX_POINTS:]

    except Exception as e:
        print(f"Error en la notificació BLE: {e}")


async def ble_task():
    print("Buscant dispositiu BLE...")
    device = await BleakScanner.find_device_by_name("TFG EMG", timeout=15.0)
    if not device:
        print("No s'ha trobat el dispositiu.")
        return

    async with BleakClient(device) as client:
        print("Connectat via BLE")
        await client.start_notify(CHARACTERISTIC_UUID, handle_notification)
        while True:
            await asyncio.sleep(0.1)

def run_ble_loop():
    asyncio.run(ble_task())


# Iniciar BLE en un fil separat
ble_thread = threading.Thread(target=run_ble_loop)
ble_thread.daemon = True
ble_thread.start()

# Iniciar gràfica
ani = animation.FuncAnimation(fig, update_plot, init_func=init, blit=True, interval=100, cache_frame_data=False)
plt.tight_layout()
plt.show()


