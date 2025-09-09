import sys
import os
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QLabel, QVBoxLayout, QHBoxLayout, QWidget,
    QGroupBox, QLineEdit, QComboBox, QFormLayout
)
from PyQt5.QtCore import Qt
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import serial

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

    def __del__(self):
        self.ser.close()

    def read_byte(self):
        while True:
            if self.ser.in_waiting > 0:
                incoming_byte = self.ser.read(1)  # Read a single byte
                return incoming_byte[0]
            
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

        # Right: Controls
        controls_layout = QVBoxLayout()

        # Control buttons
        btn_group = QGroupBox("Acciones")
        btn_layout = QVBoxLayout()
        self.sendvars_btn = QPushButton("Enviar variables")
        self.calibrate_btn = QPushButton("Calibrar")
        self.run_btn = QPushButton("Correr")
        self.stop_btn = QPushButton("Parar")
        self.export_data_btn = QPushButton("Exportar Datos")
        self.export_img_btn = QPushButton("Exportar Imagen")
        btn_layout.addWidget(self.sendvars_btn)
        btn_layout.addWidget(self.calibrate_btn)
        btn_layout.addWidget(self.run_btn)
        btn_layout.addWidget(self.stop_btn)
        btn_layout.addWidget(self.export_data_btn)
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

        # Control mode selection
        mode_group = QGroupBox("Tipo de Control")
        mode_layout = QVBoxLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["P", "PI", "PD", "PID"])
        mode_layout.addWidget(self.mode_combo)
        mode_group.setLayout(mode_layout)
        controls_layout.addWidget(mode_group)

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
        self.sendvars_btn.clicked.connect(self.send_vars)
        self.calibrate_btn.clicked.connect(self.calibrate_robot)
        self.run_btn.clicked.connect(self.run_robot)
        self.stop_btn.clicked.connect(self.stop_robot)
        self.export_data_btn.clicked.connect(self.export_data)
        self.export_img_btn.clicked.connect(self.export_image)

    def send_vars(self):
        self.status_label.setText("Estado: Enviando variables")
        self.repaint()
        # Enviar k


    def calibrate_robot(self):
        self.status_label.setText("Estado: Calibrando...")
        self.repaint()
        self.ser.write([255, 255])
        answer = self.read_byte()
        if answer == 255:
            self.status_label.setText("Estado: Calibrado")
        else:
            self.status_label.setText(f"Error: Respuesta inesperada: {answer}")
            

    def run_robot(self):
        self.status_label.setText("Estado: Corriendo...")
        # TODO: Send run command and start receiving data

    def stop_robot(self):
        self.status_label.setText("Estado: Parado")
        # TODO: Send stop command

    def export_data(self):
        # TODO: Save data to .mat file using scipy.io.savemat
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