from bleak import BleakClient, BleakScanner
from qasync import QEventLoop, asyncSlot
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtCore import QThread, pyqtSignal, QObject, pyqtSlot, QCoreApplication, QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import matplotlib.lines as mplines
import numpy as np
import sys
import os
import asyncio
import binascii
import struct
import datetime
import time
import math
TARGET_DEVICE_IDS = ["0C:EF:F6:FB:29:06", "0C:EF:F6:FB:28:E3", "0C:EF:F6:FB:28:9B", "0C:EF:F6:FB:28:D0", "0C:EF:F6:FB:28:B7", 
                     "04:E3:E5:DC:9A:D8"]
# Replace these with your actual BLE device address and UUIDs
SERVICE_UUID = "1a4e06aa-a600-4094-a0bb-698edc027190"  # Sensor Bolt Service UUID
CHARACTERISTIC_UUIDS = [    
    "00002a6e-0000-1000-8000-00805f9b34fb",  # Temperature  UUID
    "34850bc9-95e5-4701-956c-656ce1afdc66",  # Strain 1 UUID
    "ce645fc0-9d46-42ef-a89c-9699c0121c65",  # Strain 2 UUID
    "1062d25e-b825-457d-807b-9b02e2f5cb80",  # Strain 3 UUID
    "0f88a878-8b02-44cb-8fad-a317f24edaf9"   # Combined UUID
]


# Global dictionary to compensate values with a user determined initial (no calculated) offset conditions
DEVICE_ZERO_OFFSETS = {
    "0C:EF:F6:FB:29:06": {'strain_1': 0.0, 'strain_2': 0.0, 'strain_3': 0.0, 'temp': 0.0},
    "0C:EF:F6:FB:28:E3": {'strain_1': 0.0, 'strain_2': 0.0, 'strain_3': 0.0, 'temp': 0.0},
    #"0C:EF:F6:FB:28:9B": {'strain_1': -0.00016732, 'strain_2': 0.0005191, 'strain_3': 0.0004194, 'temp': 0.0},
    "0C:EF:F6:FB:28:9B": {'strain_1': 0.0, 'strain_2': 0.0, 'strain_3': 0.0, 'temp': 0.0},
    "0C:EF:F6:FB:28:D0": {'strain_1': 0.0, 'strain_2': 0.0, 'strain_3': 0.0, 'temp': 0.0},
    "0C:EF:F6:FB:28:B7": {'strain_1': 0.0, 'strain_2': 0.0, 'strain_3': 0.0, 'temp': 0.0},
    "04:E3:E5:DC:9A:D8": {'strain_1': 0.0, 'strain_2': 0.0, 'strain_3': 0.0, 'temp': 0.0}
}


def ieee754_hex_to_float(hex_value):
    int_value = int(hex_value, 16)
    float_value = struct.unpack('>f', struct.pack('@I', int_value))[0]
    return float_value


def sint16_hex_to_int(hex_value):
    int_value = int(hex_value, 16)
    signed_int_value = struct.unpack('>h', struct.pack('@H', int_value))[0]
    return signed_int_value


data_type = np.dtype([('strain_1', 'f4'),
                      ('strain_2', 'f4'),
                      ('strain_3', 'f4'),
                      ('temp', 'f4'),
                      ('timestamp', 'f4')])  # Timestamp formatted as string


# def parse_data(data, time):
#     strain_1_hex = data[:8]
#     strain_2_hex = data[8:16]
#     strain_3_hex = data[16:24]
#     temp_hex = data[24:28]
#     strain_1 = ieee754_hex_to_float(strain_1_hex)
#     strain_2 = ieee754_hex_to_float(strain_2_hex)
#     strain_3 = ieee754_hex_to_float(strain_3_hex)
#     temp = sint16_hex_to_int(temp_hex) / 100
#     # Add timestamp
#     timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
#     # Parse the data into a structured numpy array
#     parsed_data = np.array([(strain_1, strain_2, strain_3, temp, time)], dtype=data_type)
#     return parsed_data

def parse_data(data, time):
    strain_1_hex = data[:8]
    strain_2_hex = data[8:16]
    strain_3_hex = data[16:24]
    temp_hex = data[24:28]
    strain_1 = ieee754_hex_to_float(strain_1_hex)
    strain_2 = ieee754_hex_to_float(strain_2_hex)
    strain_3 = ieee754_hex_to_float(strain_3_hex)
    temp = sint16_hex_to_int(temp_hex) / 100
    parsed_data = [strain_1, strain_2, strain_3, temp, time]
    return parsed_data

# def compensate_zero_offsets(parsed_data, device_id):
#     global DEVICE_ZERO_OFFSETS
#     # Retrieve zero offsets for the given device ID
#     zero_offsets = DEVICE_ZERO_OFFSETS.get(device_id, {'strain_1': 0.0, 'strain_2': 0.0, 'strain_3': 0.0, 'temp': 0.0})

#     # Compensate the data for zero offsets
#     compensated_data = np.array([(
#         parsed_data['strain_1'][0] - zero_offsets.get('strain_1', 0.0),
#         parsed_data['strain_2'][0] - zero_offsets.get('strain_2', 0.0),
#         parsed_data['strain_3'][0] - zero_offsets.get('strain_3', 0.0),
#         parsed_data['temp'][0] - zero_offsets.get('temp', 0.0),
#         parsed_data['timestamp'][0])], dtype=data_type)

#     return compensated_data


class BLEDataPlotter(QMainWindow):
    
    plot_update_signal = pyqtSignal(str, np.ndarray, np.ndarray)
    scanner_signal = pyqtSignal()
    ble_task = None
    def __init__(self):
        super().__init__()
        self.setGeometry(100, 100, 1700, 900)
        self.setWindowTitle("BLE Data Plotter")
        self.ScannerTask = None
        
        self.scanner_signal.connect(self.ScannerStop)
        self.scanner = BleakScanner(self.detection_callback)
        # Using QTabWidget to separate plots for each device and making it the central widget
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        #Adding the Menu Bar with the search command
        menu_bar = self.menuBar()
        startAction = QAction('Search Devices', self)
        startAction.triggered.connect(self.ScannerRun)
        menu_bar.addAction(startAction)

        # Create a dictionary to store plot widgets for each device 
        self.device_plotters = {}

       
        self.show()

    def add_device_tab(self, address):
        plot_widget = DevicePlotTab(address)
        self.device_plotters[address] = plot_widget #Here the device addres is added and associated its widget
        self.tabs.addTab(plot_widget, f"Device {address}")
        #self.ble_signal.emit(address)


    def detection_callback(self, device, advertisement_data):
        if device.address in TARGET_DEVICE_IDS and device.address not in self.device_plotters: #only executes if the deviced is not alreay added
            print(f"Device {device.address} detected")
            # Extract manufacturer data
            manufacturer_data = advertisement_data.manufacturer_data                  
            if manufacturer_data:
                # Convert manufacturer data to hex string
                manufacturer_data_hex = {k: binascii.hexlify(v).decode('utf-8') for k, v in manufacturer_data.items()}
                # Print the device details
                print(f"Target Device Detected: {device.address} | RSSI: {device.details} | Name: {device.name} | Manufacturer Data (Hex): {manufacturer_data_hex}")
                # Process the manufacturer data if it's in the expected format
                for manufacturer_id, data_hex in manufacturer_data_hex.items():
                    try:
                        self.add_device_tab(device.address)
                    except ValueError as e:
                        print(f"Error parsing manufacturer data: {e}")
            self.scanner_signal.emit()
            print("\n")

    
    # @asyncSlot()
    # async def TestBLEWork(self):
    #     while True:
    #         self.BLEWorkerSignal.emit("0C:EF:F6:FB:29:06", np.random.rand(1,5), np.random.rand(1,5))
    #         await asyncio.sleep(60)
    
    #This function scans constantly for devices registered in TARGET_DEVICE_IDS
    @asyncSlot()
    async def ScannerWork(self):
        self.ScannerTask= asyncio.create_task(self.ScannerRun())
        try:
            await self.ScannerTask
        except asyncio.CancelledError:
            print("Scanner Proccess Ended")
            
    @asyncSlot()
    async def ScannerRun(self):
        #while True:
        await self.scanner.start()
        print("Scanner Started")
        #await asyncio.sleep(60)
    @asyncSlot()
    async def ScannerStop(self):
        #while True:
        await self.scanner.stop()
        print("Scanner Stopped")

   
class BarThread(QThread):
    progress_signal = pyqtSignal(int)
    def __init__(self, parent = None):
        super().__init__(parent)
        self.is_running = True

    def run(self):
        cnt = 0
        while(self.is_running):
            cnt += 1

            if cnt == 100:
                cnt = 0
            
            self.progress_signal.emit(cnt)
            time.sleep(0.01)

    def stop(self):
        self.is_running = False
        self.terminate()

        
    
class DevicePlotTab(QWidget):
    def __init__(self, device_address):
        super().__init__()
        self.device_address = device_address
        self.ble_task = None
        #self.worker = None
        #self.worker_thread = None
        self.c1 = 0.01
        self.c2 = 0.01
        self.c3 = 0.01
        self.inputDialogConstants = None
        self.task_running = False
        self.window_span = 1000
        self.window_step= 100
        self.plotter_count = 0
        self.time_record = 0
        #define initial offsets
        self.strain_offsets = [0, 0, 0]
        self.progress_thread = None
        self.informative_bar = QProgressBar()

        # Create a layout -- Device-specific plot

        #Crate Widget Layout in Self (QWidget)
        self.main_layout = QVBoxLayout(self)

        # Create a matplotlib figure and canvas
        self.figure = plt.Figure(constrained_layout=True)
        self.canvas = FigureCanvas(self.figure)
        self.main_layout.addWidget(self.canvas)

        
        
       

        # Button layout for buttons in the Widget
        button_layout = QHBoxLayout()
        #Calibrate Button
        zero_button = QPushButton('Zero Calibrate Strains')
        zero_button.clicked.connect(self.zero_calibrate)  # Connect the button to the zero calibration function
        button_layout.addWidget(zero_button)
        #Clear Button
        clear_button = QPushButton('Clear Data')
        #clear_button.clicked.connect(self.clear_data)  # Connect the button to the clear data function
        button_layout.addWidget(clear_button)
        # Connect Button
        self.connect_button = QPushButton('Connect')
        self.connect_button.clicked.connect(self.schedule_ble_task)  # Connect to function
        button_layout.addWidget(self.connect_button)
        # Disconnect Button
        disconnect_button = QPushButton('Disconnect')
        #disconnect_button.clicked.connect(self.stop_ble_connection)  # Disconnect function
        button_layout.addWidget(disconnect_button)

        # set constnats Button
        constats_button = QPushButton('Set Constants')
        constats_button.clicked.connect(self.set_constants)  # Connect to function
        button_layout.addWidget(constats_button)

        #Finally ad layout
        self.main_layout.addLayout(button_layout)

        # Initial plot -- Axes for STrains

        #General
        self.ax = self.figure.add_subplot(1,2,1)
        self.ax.set_title(f'Device {device_address} - Real-time BLE Data', fontsize=10)
        
        #x axis
        self.ax.set_xlabel('Time', fontsize=8)
        #self.ax.tick_params(axis='x', rotation=45)
        self.ax.tick_params(axis='y', direction='in',  labelleft=True)
        #y axis
        self.ax.set_ylabel('Strain Values (mV/V)', fontsize=8)  # Change to µV/V
        self.ax.set_ylim(-3, 3)  # Set initial strain value limits (in µV/V)

        # Create a second y-axis for the temperature
        self.ax2 = self.ax.twinx()
        self.ax2.set_ylabel('Temperature (°C)', rotation=270, fontsize=8)
        # Move the temperature axis ticks and label to the right side
        self.ax2.yaxis.set_label_position("right")
        self.ax2.tick_params(axis='y', direction='in')
        self.ax2.yaxis.tick_right()
        self.ax2.set_ylim(-40, 105)  # Limit temperature axis between -40 and 105

        
        self.ax_f = self.figure.add_subplot(1,2,2)
        self.ax_f.set_title('Axial Force', fontsize=10)
        #self.ax_f.tick_params(axis='x', rotation=45)
        self.ax_f.tick_params(axis='y', direction='in')
        self.ax_f.set_ylabel('Force [kN]', fontsize=8)  
        self.ax_f.set_ylim(-1, 1)
        
        self.ax_f.set_xlabel('Time', fontsize=8)
        
         # Create a second y-axis for the temperature
        self.ax_f_2 = self.ax_f.twinx()
        self.ax_f_2.set_ylabel('Moment (Nm)', rotation=270, fontsize=8)
        # Move the temperature axis ticks and label to the right side
        self.ax_f_2.yaxis.set_label_position("right")
        self.ax_f_2.tick_params(axis='y', direction='in')
        self.ax_f_2.yaxis.tick_right()
        self.ax_f_2.set_ylim(-40, 105) 
        
        #self.figure.subplots_adjust(wspace=0.8)
        
    

        # Initialize data arrays for real-time plotting
        self.time_data = []
        self.strain_1_data = []
        self.strain_2_data = []
        self.strain_3_data = []
        self.strain_1_data_raw = []
        self.strain_2_data_raw = []
        self.strain_3_data_raw = []
        self.temp_data = []

        self.f_ax_data = []
        
        self.mx = []
        self.my = []
        self.m_total =[]
         # Store raw (uncompensated) data for zero calibration

        # Initialize lines for the data
        self.strain_1_line = None
        self.strain_2_line = None
        self.strain_3_line = None
        self.temp_line = None

        self.f_ax_data_line = None


        #Initialize Timer for plotting

        # self.timer = QTimer()
        # self.timer.setInterval(30)
        # self.timer.timeout.connect(self.update_plot)
        # self.plot_timer_started = False
    
    
    def start_progress_bar_work(self):
        
        self.main_layout.insertWidget(len(self.main_layout) - 1, self.informative_bar)
        self.progress_thread = BarThread()
        self.informative_bar.setFormat(f"Tryng To Connect To Device {self.device_address}")
        self.progress_thread.progress_signal.connect(self.update_bar)
        self.progress_thread.start()
        pass
    
    def stop_progress_bar_work(self):
        self.progress_thread.stop()
        self.main_layout.removeWidget(self.informative_bar)
        # Hide and delete the widget (optional)
        self.informative_bar.setParent(None) 
        self.informative_bar.deleteLater()
        
        
    @pyqtSlot(int)
    def update_bar(self, cnt):
        self.informative_bar.setValue(cnt)

    def update_data(self, parsed_data):
        # # Extract timestamp and append it
        #timestamp_str = compensated_data['timestamp'][0]
        #timestamp = datetime.datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S.%f").microsecond
        self.time_data.append(parsed_data[4])
        # # Append uncompensated strain values for zero calibration
        self.strain_1_data_raw.append(parsed_data[0])
        self.strain_2_data_raw.append(parsed_data[1])
        self.strain_3_data_raw.append(parsed_data[2])


        compensated_data = [x1 - x2 for (x1, x2) in zip(parsed_data[0:3], self.strain_offsets)]

        
        print(compensated_data)
        # # Append compensated strain values for plotting, multiplied by 10^6 to convert to µV/V
        self.strain_1_data.append(compensated_data[0]* 1e3)
        self.strain_2_data.append(compensated_data[1]* 1e3)
        self.strain_3_data.append(compensated_data[2]* 1e3)
        self.temp_data.append(parsed_data[3])
        
        
        #c1 c2 c3 are constant or can be modified through main window
        f_ax_data_temp =( (compensated_data[0] /self.c1) +
        (compensated_data[1]/ self.c2 )+ (compensated_data[2]/ self.c3)) / (3)

        self.f_ax_data.append(f_ax_data_temp)
        
        basis_x = [(compensated_data[0] /self.c1), -(compensated_data[1]/ self.c2 ), (compensated_data[2]/ self.c3)]
        basis_y = [(compensated_data[0] /self.c1), -(compensated_data[1]/ self.c2 ), -(compensated_data[2]/ self.c3)]
        
        Lx = [0, 4.33, 4.33]
        Ly = [5, 0.0025, 0.0025]
        
        m_x = sum(basis_x[i]*Lx[i] for i in range(len(basis_x)))
        m_y = sum(basis_y[i]*Ly[i] for i in range(len(basis_y)))
        self.mx.append(m_x)
        self.my.append(m_y)
        self.m_total.append(math.sqrt((m_x*m_x)+(m_y*m_y)))

        #Start plotting only from 90 values
        # if len(self.time_data) > self.window_span and not self.plot_timer_started:
        #     self.timer.start()
        #     self.plot_timer_started = True

        self.update_plot()
        
    def update_plot(self):
        if (len(self.time_data) % self.window_step == 0): #every step (100)
            if len(self.time_data) >= self.window_span+(self.plotter_count*self.window_step): #depending won whether the window span is reached
                _from = self.plotter_count* self.window_step
                _end = self.window_span+(self.plotter_count*self.window_step)
                self.plotter_count += 1
            elif len(self.time_data) < self.window_span:
                _from = 0
                _end = len(self.time_data)
            else:
                return
            
            window = (_from, _end)
            window_slice = slice(*window)

            #self.ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S')) # Format x-axis to show only hour, minute, second
            # Adjust strain y-axis limits if necessar
            min_strain = min(min(self.strain_1_data[window_slice]), min(self.strain_2_data[window_slice]), min(self.strain_3_data[window_slice]))
            max_strain = max(max(self.strain_1_data[window_slice]), max(self.strain_2_data[window_slice]), max(self.strain_3_data[window_slice]))
            #if min_strain < -3 or max_strain > 3:  # Adjust limits in µV/V
            self.ax.set_ylim(min_strain-0.3, max_strain+0.3)
            #else:
                #self.ax.set_ylim(-3, 3)  # Default range in µV/V
            min_force = min(self.f_ax_data[window_slice])
            max_force = max(self.f_ax_data[window_slice])
            self.ax_f.set_ylim(min_force-0.1, max_force+0.1)
                  
            min_moment = min(min(self.mx[window_slice]), min(self.my[window_slice]), min(self.m_total[window_slice]))
            max_moment = max(max(self.mx[window_slice]), max(self.my[window_slice]), max(self.m_total[window_slice]))
            self.ax_f_2.set_ylim(min_moment-0.2, max_moment+0.2)
            
            if self.temp_line is None:
            # # Limit data to last 90 entries for better visualizatio
                self.strain_1_line = self.ax.plot(self.time_data[window_slice], self.strain_1_data[window_slice], label="Strain 1")
                self.strain_2_line = self.ax.plot(self.time_data[window_slice], self.strain_2_data[window_slice], label="Strain 2")
                self.strain_3_line = self.ax.plot(self.time_data[window_slice], self.strain_3_data[window_slice], label="Strain 3")
                self.ax.legend(loc='upper left', fontsize=10)
                self.temp_line = self.ax2.plot(self.time_data[window_slice], self.temp_data[window_slice], color='r', label="Temperature")
                self.ax2.legend(loc='upper right', fontsize=10)
                self.f_ax_data_line = self.ax_f.plot(self.time_data[window_slice], self.f_ax_data[window_slice], label="Total Force")
                self.ax_f.legend(loc='upper left', fontsize=10)
                self.mx_line = self.ax_f_2.plot(self.time_data[window_slice], self.mx[window_slice], color='r', label="Moment x")
                self.my_line = self.ax_f_2.plot(self.time_data[window_slice], self.my[window_slice], color='m', label="Moment y")
                self.m_total_line = self.ax_f_2.plot(self.time_data[window_slice], self.m_total[window_slice], color='g', label="Total moment")
                self.ax_f_2.legend(loc='upper right', fontsize=10)
                
            else:

                self.strain_1_line[0].set_data(self.time_data[window_slice], self.strain_1_data[window_slice])
                self.strain_2_line[0].set_data(self.time_data[window_slice], self.strain_2_data[window_slice])
                self.strain_3_line[0].set_data(self.time_data[window_slice], self.strain_3_data[window_slice])
                self.temp_line[0].set_data(self.time_data[window_slice], self.temp_data[window_slice])
                self.f_ax_data_line[0].set_data(self.time_data[window_slice], self.f_ax_data[window_slice])
                self.mx_line[0].set_data(self.time_data[window_slice], self.mx[window_slice])
                self.my_line[0].set_data(self.time_data[window_slice], self.my[window_slice])
                self.m_total_line[0].set_data(self.time_data[window_slice], self.m_total[window_slice])
                self.ax.set_xlim(window)
                self.ax2.set_xlim(window)
                self.ax_f.set_xlim(window)
                self.ax_f_2.set_xlim(window)
                #Redraw the canvas
                self.canvas.draw()
                
        else:
            return

    @pyqtSlot()
    def zero_calibrate(self):
        #calculate ofsset for update data
        #if len(self.strain_1_data_raw)>100:
            self.strain_offsets[0] = sum(self.strain_1_data_raw[-100:])/100
            self.strain_offsets[1] = sum(self.strain_2_data_raw[-100:])/100
            self.strain_offsets[2] = sum(self.strain_3_data_raw[-100:])/100
        
    
    @pyqtSlot()
    def set_constants(self):
        self.inputDialogConstants = InputDialogConstants()
        self.inputDialogConstants.exec()
        try:
            self.c1, self.c2, self.c3 = self.inputDialogConstants.getInputs()
            print(self.inputDialogConstants.getInputs())
        except TypeError:
            print("No constnats given/Incorrect input format")
            
    @asyncSlot()
    async def schedule_ble_task(self):
        self.ble_task = asyncio.create_task(self.ble_task_run())
        self.start_progress_bar_work()
        self.connect_button.setEnabled(False)
        try:
            await self.ble_task
            #await self.update_task
        except asyncio.CancelledError:
            print("BLE Task Cancelled")
    
    async def notification_handler(self, sender, data):
        # Extract the UUID from the sender, assuming it is part of a tuple or object
        sender_uuid = str(sender)  # Ensure sender is converted to a string representation
        self.time_record +=1
        if CHARACTERISTIC_UUIDS[0] in sender_uuid:  # Temperature UUID
            print(f"Notification from {sender}: {sint16_hex_to_int(data.hex())/100} (raw: {data})")
        elif any(uuid in sender_uuid for uuid in CHARACTERISTIC_UUIDS[1:3]):  # Strain UUIDs        
            print(f"Notification from {sender}: {ieee754_hex_to_float(data.hex())} (raw: {data})")
        elif any(uuid in sender_uuid for uuid in CHARACTERISTIC_UUIDS[4]):  # Combined UUID
            parsed_data = parse_data(data.hex(), self.time_record)
            self.update_data(parsed_data)   
            print(f"{parsed_data}") 
        else:
            print(f"Notification from {sender}: Unknown characteristic (raw: {data})")

    async def ble_task_run(self):
        """Asynchronous BLE task that can be stopped"""
        while True:  # Endlosschleife für Reconnect
            print(f"Trying to connect to {self.device_address}")
            
            try:
                async with BleakClient(self.device_address, timeout=60) as client:
                    if client.is_connected:
                        print(f"Connected to {self.device_address}")
                        self.stop_progress_bar_work()
                        #self.start_data_worker()
                        # Subscribe to notifications for all characteristics
                        print("Subscribing to notifications for all characteristics...")
                        try:
                            for char_uuid in CHARACTERISTIC_UUIDS:
                                await client.start_notify(char_uuid, self.notification_handler)
                            # Keep the connection alive to receive notifications
                            self.task_running = True
                            print("Waiting for notifications. Press Ctrl+C to stop.")
                            while True:
                                if not client.is_connected:  # Verbindung prüfen
                                    print("Connection lost. Attempting to reconnect...")
                                    break  # Verlasse die Schleife, um die Verbindung neu aufzubauen
                                await asyncio.sleep(20)  # Anpassbare Wartezeit
                        except asyncio.CancelledError:
                            print("Stopping notifications...")
                            for char_uuid in SERVICE_UUID:
                                await client.stop_notify(char_uuid)
                            self.task_running = False
            except Exception as e:
                print(f"BLE Connection Error: {e}")
            print("Reconnecting in 2 seconds...")
            await asyncio.sleep(2)  # Wartezeit vor dem erneuten Versuch

    def stop_ble_task(self):
        """Stop the BLE connection"""
        self.ble_task.cancel()


class InputDialogConstants(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Type constants")
        self.c1 = QLineEdit(self)
        self.c1.setValidator(QDoubleValidator(0.00, 99.99, 2))
        self.c1.setText("0.01")
        self.c2 = QLineEdit(self)
        self.c2.setText("0.01")
        self.c2.setValidator(QDoubleValidator(0.00, 99.99, 2))
        self.c3 = QLineEdit(self)
        self.c3.setText("0.01")
        self.c3.setValidator(QDoubleValidator(0.00, 99.99, 2))
        self.buttonBox = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, self)
        layout = QFormLayout(self)
        layout.addRow("C1", self.c1)
        layout.addRow("C2", self.c2)
        layout.addRow("C3", self.c3)
        layout.addWidget(self.buttonBox)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)
    def getInputs(self):
        try:
            return (float(self.c1.text()), float(self.c2.text()), float(self.c3.text()))
        except Exception as e:
            print(e)

        


if __name__ == '__main__':
    app = QApplication(sys.argv)
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)

    mainWindow = BLEDataPlotter()

    with loop:
        sys.exit(loop.run_forever())
    

