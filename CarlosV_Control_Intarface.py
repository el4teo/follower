import sys
import os
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QLabel, QVBoxLayout, QHBoxLayout, QWidget,
    QGroupBox, QLineEdit, QComboBox, QFormLayout
)
from PyQt5.QtCore import QTimer
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import serial
import time
import struct
import numpy as np


DATA_DIR = "data"
IMAGES_DIR = "images"

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CarlosV Control Interface")
        self.setGeometry(100, 100, 1000, 600)
        self.initUI()
        self.bluetooth_client = None
        self.ser = serial.Serial('COM3', 9600, timeout=1)
        self.buffer_serial = bytearray()

    def __del__(self):
        self.ser.close()

    def read_byte(self):
        start_time = time.time()
        while True:
            if self.ser.in_waiting > 0:
                incoming_byte = self.ser.read(1)  # Read a single byte
                return incoming_byte[0]
            if time.time() - start_time >= 5: # timeout
                self.status_label.setText("Warning: Timeout happened!")
                self.repaint()
                time.sleep(1)
                return None  # or raise an exception
            
    def initUI(self):
        # Main horizontal layout
        central_widget = QWidget()
        main_layout = QHBoxLayout()

        # Left: Plot area
        plot_group = QGroupBox("SEGUIDOR DE LINEAS")
        plot_layout = QVBoxLayout()
        self.figure = Figure(figsize=(6, 4))
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("SEGUIDOR DE LINEAS")
        self.ax.set_xlabel("Tiempo (s)")
        self.ax.set_ylabel("Sensor / Motores")
        plot_layout.addWidget(self.canvas)
        plot_group.setLayout(plot_layout)
        main_layout.addWidget(plot_group, stretch=2)

        # Default lines
        n = 501
        T = 10e-3
        x = np.linspace(-1* T * (n - 1), 0, n)
        y = np.zeros(n)
        self.ax.set_xlim(x[0], x[-1])
        self.ax.set_ylim(-100, 100)
        # Default ax configuration
        self.ax.set_facecolor("#fff7df") # lightgoldenrodyellow
        self.ax.grid(True, which='major', linestyle='--', linewidth=0.7, color='black')
        self.hPlotLeftMotor = self.ax.plot(x, y, label="left pow")
        self.hPlotSensor = self.ax.plot(x, y, label="line pos")
        self.hPlotRightMotor = self.ax.plot(x, y, label="right pow")
        self.ax.legend()
        self.canvas.draw()

        # Timer
        self.start_time = time.time()
        self.timer = QTimer()
        self.timer.setInterval(100)  # 100 ms = 10 Hz de refresco
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

        # Right: Controls
        controls_layout = QVBoxLayout()

        # Control buttons
        btn_group = QGroupBox("Acciones")
        btn_layout = QVBoxLayout()
        self.updategraph_btn = QPushButton("Graficar")
        self.calibrate_btn = QPushButton("Calibrar")
        self.run_btn = QPushButton("Correr")
        self.stop_btn = QPushButton("Parar")
        self.restart_btn = QPushButton("Reiniciar")
        self.export_img_btn = QPushButton("Exportar Imagen")
        btn_layout.addWidget(self.updategraph_btn)
        btn_layout.addWidget(self.calibrate_btn)
        btn_layout.addWidget(self.run_btn)
        btn_layout.addWidget(self.stop_btn)
        btn_layout.addWidget(self.restart_btn)
        btn_layout.addWidget(self.export_img_btn)
        btn_group.setLayout(btn_layout)
        controls_layout.addWidget(btn_group)

        # Control variables
        var_group = QGroupBox("Variables de Control")
        var_layout = QFormLayout()
        self.k_input = QLineEdit("0.25")
        self.ti_input = QLineEdit("1")
        self.td_input = QLineEdit("0.05")
        self.t_input = QLineEdit("0.02")
        self.pot_max_input = QLineEdit("80")
        self.pot_min_input = QLineEdit("-20")
        self.pot_ref_input = QLineEdit("30")
        var_layout.addRow("K:", self.k_input)
        var_layout.addRow("Ti:", self.ti_input)
        var_layout.addRow("Td:", self.td_input)
        var_layout.addRow("T:", self.t_input)
        var_layout.addRow("PotMax:", self.pot_max_input)
        var_layout.addRow("PotMin:", self.pot_min_input)
        var_layout.addRow("PotRef:", self.pot_ref_input)
        var_group.setLayout(var_layout)
        controls_layout.addWidget(var_group)
        ## Callbacks
        self.k_input.editingFinished.connect(self.send_k)
        self.ti_input.editingFinished.connect(self.send_Ti)
        self.td_input.editingFinished.connect(self.send_Td)
        self.t_input.editingFinished.connect(self.send_T)
        self.pot_max_input.editingFinished.connect(self.send_MaxPow)
        self.pot_min_input.editingFinished.connect(self.send_MinPow)
        self.pot_ref_input.editingFinished.connect(self.send_RefPow)

        # Control mode selection
        mode_group = QGroupBox("Tipo de Control")
        mode_layout = QVBoxLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["P", "PI", "PD", "PID"])
        mode_layout.addWidget(self.mode_combo)
        mode_group.setLayout(mode_layout)
        controls_layout.addWidget(mode_group)
        ## Callback
        self.mode_combo.currentIndexChanged.connect(self.run_robot)


        # Status label at the bottom
        self.status_label = QLabel("Estado: Sin calibrar")
        controls_layout.addWidget(self.status_label)

        # Add controls to main layout
        controls_widget = QWidget()
        controls_widget.setLayout(controls_layout)
        main_layout.addWidget(controls_widget, stretch=1)

        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        # Connect signals
        self.updategraph_btn.clicked.connect(self.update_plot)
        self.calibrate_btn.clicked.connect(self.calibrate_robot)
        self.run_btn.clicked.connect(self.run_robot)
        self.stop_btn.clicked.connect(self.stop_robot)
        self.restart_btn.clicked.connect(self.restart)
        self.export_img_btn.clicked.connect(self.export_image)

    def send_k(self):
        # k Factor  252 + int32
        k = int(float(self.k_input.text()) * 1000)
        print(f"Enviando k = {k}")
        self.repaint()
        self.ser.write([252])
        bytes = struct.pack('<I', k)
        self.ser.write(bytes)

    def send_Ti(self):
        # Ti        251 + int32
        ti = int(float(self.ti_input.text()) * 1000)
        print(f"Enviando Ti = {ti}")
        self.repaint()
        self.ser.write([251])
        bytes = struct.pack('<I', ti)
        self.ser.write(bytes)

    def send_Td(self):
        # Td        250 + int32
        td = int(float(self.td_input.text()) * 1000)
        print(f"Enviando Td = {td}")
        self.repaint()
        self.ser.write([250])
        bytes = struct.pack('<I', td)
        self.ser.write(bytes)
        
    def send_T(self):
        # T         243 + [1, 200]
        t = int(float(self.t_input.text()) * 100)
        print(f"Enviando T = {t}")
        self.repaint()
        self.ser.write([243])
        bytes = struct.pack('<I', t)
        self.ser.write(bytes)

    def send_MaxPow(self):
        # Max Power 254 + [0, 200]
        max_pow = int(self.pot_max_input.text())
        print(f"Enviando MaxPower = {max_pow}")
        self.repaint()
        self.ser.write([254, max_pow + 100])

    def send_MinPow(self):
        # Min Power 253 + [0, 200]
        min_pow = int(self.pot_min_input.text())
        print(f"Enviando MinPower = {min_pow}")
        self.repaint()
        self.ser.write([253, min_pow + 100])

    def send_RefPow(self):
        # Ref power 249 + [0, 200]
        ref_pow = int(self.pot_ref_input.text())
        print(f"Enviando RefPower = {ref_pow}")
        self.repaint()
        self.ser.write([249, ref_pow + 100])

    def decode_serial_data(self):
        if self.ser.in_waiting > 0:
            incoming_data = self.ser.read(self.ser.in_waiting)
            self.buffer_serial.extend(incoming_data)
        else:
            return np.array([]), np.array([]), np.array([])
        
        data = self.buffer_serial  # make it iterable by ints
        n_max = len(data)

        sensors = []
        lefts = []
        rights = []

        i = 0
        while i <= n_max - 4:  # ensure at least 4 bytes remain
            frame = data[i:i+4]
            if frame[0] != 201:
                # not aligned → shift by one and try again
                i += 1
                continue
            sensors.append(frame[1] - 100)
            lefts.append(frame[2] - 100)
            rights.append(frame[3] - 100)
            i += 4  # jump to next potential frame

        # Store leftover (incomplete) bytes for next call
        self.buffer_serial = bytearray(data[i:])

        return np.array(lefts), np.array(sensors), np.array(rights)
    
    def update_plot(self):
        # Get new values from serial
        y_left_new, y_sensor_new, y_right_new = self.decode_serial_data()
        n_new = len(y_left_new)


        if n_new == 0:
            return  # nothing new, skip update
        elif n_new < 501:
            # Current y-data in the plots (numpy arrays)
            y_left_old   = self.hPlotLeftMotor[0].get_ydata()
            y_sensor_old = self.hPlotSensor[0].get_ydata()
            y_right_old  = self.hPlotRightMotor[0].get_ydata()
            # Drop oldest n_new samples and append the new ones
            y_left   = np.concatenate([y_left_old[n_new:],   y_left_new])
            y_sensor = np.concatenate([y_sensor_old[n_new:], y_sensor_new])
            y_right  = np.concatenate([y_right_old[n_new:],  y_right_new])
        else:
            # Keep only the latest 501 samples
            y_left   = y_left_new[-501:]
            y_sensor = y_sensor_new[-501:]
            y_right  = y_right_new[-501:]

        # Update the internal y-data arrays
        self.hPlotLeftMotor[0].set_ydata(y_left)
        self.hPlotSensor[0].set_ydata(y_sensor)
        self.hPlotRightMotor[0].set_ydata(y_right)

        # Redibujar en canvas
        self.canvas.draw()

    def calibrate_robot(self):
        self.status_label.setText("Estado: Calibrando...")
        self.repaint()
        self.ser.write([255, 255])
        answer = self.read_byte()
        if answer == 255:
            self.status_label.setText("Estado: Calibrado")
        else:
            self.status_label.setText(f"Error: Respuesta inesperada: {answer}")
        
        # Enviando variables
        self.repaint()
        self.status_label.setText("Estado: Enviando variables")
        self.repaint()
        self.send_k()
        self.send_Ti()
        self.send_Td()
        self.send_T()
    
        self.send_MaxPow()
        self.send_MinPow()
        self.send_RefPow()

    def run_robot(self):
        # Restart   255 + 0
        # Stop      248
        # P         247
        # PI        246
        # PD        245
        # PID       244
        mode_value = self.mode_combo.currentText()
        if mode_value == "P":
            self.ser.write([247])
        elif mode_value == "PI":
            self.ser.write([246])
        elif mode_value == "PD":
            self.ser.write([245])
        elif mode_value == "PID":
            self.ser.write([244])
        self.status_label.setText(f"Estado: Corriendo en {mode_value}")


    def stop_robot(self):
        self.ser.write([248])

    def restart(self):
        self.ser.write([255, 0])
        pass

    def export_image(self):
        img_path = os.path.join(IMAGES_DIR, "exported_plot.jpg")
        self.figure.savefig(img_path)
        self.status_label.setText(f"Imagen exportada: {img_path}")

if __name__ == "__main__":
    if not os.path.exists(DATA_DIR):
        os.makedirs(DATA_DIR)
    if not os.path.exists(IMAGES_DIR):
        os.makedirs(IMAGES_DIR)

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())