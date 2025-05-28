 import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# 🔧 CONFIGURA segons el teu sistema:
PORT = 'COM5'        # ⚠️ Canvia-ho pel teu port (Linux: '/dev/ttyUSB0' o '/dev/ttyACM0')
BAUDRATE = 115200
MAX_POINTS = 1000    # Nombre màxim de mostres visibles

# 🎯 Inicia buffer circular
data = deque([0.0]*MAX_POINTS, maxlen=MAX_POINTS)

# 🔌 Obre el port sèrie
ser = serial.Serial(PORT, BAUDRATE, timeout=1)

# 📊 Inicialitza la gràfica
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=1, label="Canal 1")

ax.set_title("Senyal EMG en temps real – Canal 1")
ax.set_xlabel("Mostres")
ax.set_ylabel("Amplitud (mV)")
ax.set_xlim(0, MAX_POINTS)
ax.set_ylim(-1, 2)  # Ajusta segons el rang esperat del teu senyal
ax.grid(True)
ax.legend()

# 🔁 Funció d’actualització de la gràfica
def update(frame):
    while ser.in_waiting:
        try:
            line_bytes = ser.readline()
            line_str = line_bytes.decode('utf-8').strip()
            valor = float(line_str)
            data.append(valor)
        except Exception as e:
            print(f"Error: {e}")
            pass

    line.set_data(range(len(data)), data)
    return line,

# ▶️ Inicia l’animació
ani = animation.FuncAnimation(fig, update, interval=10, blit=True)
plt.tight_layout()
plt.show()
