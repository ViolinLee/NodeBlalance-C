import os
import sys
import cv2
import _thread
import sqlite3
import numpy as np
import socket
from serial.tools import list_ports
from glob import glob
from datetime import datetime
from audio_detector import AudioDetector
from face_detector import CascadeDetector
from PySide2 import QtGui, QtWidgets, QtCore
from PySide2.QtCore import QDir, QTimer, QSize
from PySide2.QtGui import QPixmap, QImage
from PySide2.QtWidgets import QMainWindow, QFileDialog, QMessageBox, QDialog, QLineEdit, QDialogButtonBox, QFormLayout
from device_connector import SerialDeviceHandler, CamWSClientCallback, BrainWSClientAsync, UDPSocketCallback
from nb_tool_ui import Ui_MainWindow
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib
matplotlib.use('Qt5Agg')


class IPInputDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.first = QLineEdit("192.168.0.102")
        self.second = QLineEdit("192.168.0.108")
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, self)

        layout = QFormLayout(self)
        layout.addRow("Camera IP", self.first)
        layout.addRow("CarCmd IP", self.second)
        layout.addWidget(button_box)

        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)

    def get_inputs(self):
        return self.first.text(), self.second.text()


class MplCanvas(FigureCanvas):
    def __init__(self, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.pwd = os.getcwd()
        # self.conn = sqlite3.connect('nb_tool.db')
        self.audio_detector = AudioDetector('fully_trained_model.h5')
        self.face_detector = CascadeDetector('haarcascade_frontalface_alt.xml')
        self.latest_telemetry_data = [0, 0, 0]
        self.is_recording = 0
        self.is_lamp_toggle = 0
        self.car_cmd_mode = 0
        self.flash_mode = 0
        self.ip_addr = socket.gethostbyname(socket.gethostname())
        self.serial_port = None
        self.serial_device = SerialDeviceHandler(self.telemetry_queue_update)  # 用作传感数据接收(esp32)
        self.brain_ws_client = BrainWSClientAsync()  # 用作指令发送(esp32)
        self.cam_ws_client = CamWSClientCallback(self.show_websocket_cam_stream)  # 用作图传接收(esp32-cam)
        self.audio_udp_client = UDPSocketCallback(self.handle_audio_stream)
        self.audio_udp_client.sock.bind((self.ip_addr, 3333))
        self.audio_udp_client.start()
        # self.serial_device.configure_comm("COM5")
        # self.brain_ws_client.configure_conn("ws://192.168.0.110/RobotInput")
        # self.cam_ws_client.configure_conn("ws://192.168.0.102/Camera")

        self.canvas = MplCanvas(width=5, height=4, dpi=100)
        sample_num = 50
        self.xdata = list(range(sample_num))
        self.ydata = [0 for i in range(sample_num)]
        self._plot_ref = None
        self.update_monitor_plot()
        self.show()
        self.timer = QTimer()  # Setup a timer to trigger the redraw by calling update_plot.
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_monitor_plot)
        self.timer.start()

    def window_init(self):
        # 调试信息
        print("本机地址: {}".format(self.ip_addr))

        # 自适应窗口缩放
        self.image_label.setScaledContents(True)
        # self.plottingGraphicsView.setScaledContents(True)

        self.left_button.clicked.connect(lambda: self.on_button('left'))  # 控制按钮
        self.right_button.clicked.connect(lambda: self.on_button('right'))
        self.forward_button.clicked.connect(lambda: self.on_button('forward'))
        self.backward_button.clicked.connect(lambda: self.on_button('backward'))
        self.stop_button.clicked.connect(lambda: self.on_button('stop'))

        self.ip_modify_button.clicked.connect(lambda: self.on_button('connect_board'))

        self.speed_slider.valueChanged.connect(self.set_moving_speed)

        self.constant_button.clicked.connect(self.toggle_lamp)
        self.flash_button.toggled.connect(self.toggle_flash_mode)  # 改 radio button

        self.action_normal.toggled.connect(lambda: self.on_button('normal'))  # action group
        self.action_voice.toggled.connect(lambda: self.on_button('voice'))
        self.action_clingy.toggled.connect(lambda: self.on_button('clingy'))

        self.capture_button.clicked.connect(lambda: self.on_button('capture'))
        self.recording_button.clicked.connect(lambda: self.on_button('recording'))

        self.detect_button.clicked.connect(self.port_detect)  # 串口检测

        self.connect_button.setEnabled(False)
        self.connect_button.clicked.connect(lambda: self.on_button('connect'))

    def update_monitor_plot(self):
        # Drop off the first y element, append a new one.
        self.ydata = self.ydata[1:] + [self.latest_telemetry_data[0]]

        if self._plot_ref is None:
            plot_refs = self.canvas.axes.plot(self.xdata, self.ydata, 'r')
            self._plot_ref = plot_refs[0]
        else:
            # We have a reference, we can use it to update the data for that line.
            self._plot_ref.set_ydata(self.ydata)

        # Trigger the canvas to update and redraw.
        self.canvas.draw()

    def telemetry_queue_update(self, telemetry):
        self.latest_telemetry_data = telemetry

    def show_websocket_cam_stream(self, img_str):
        self.latest_image = cv2.imdecode(np.fromstring(img_str, np.uint8), cv2.IMREAD_COLOR)
        if self.car_cmd_mode == 2:
            img2show, bbox = self.face_detector.predict_cv2img(self.latest_image)
            self.show_image_cv2(self.image_label, img2show)

            if len(bbox) >= 1:
                img_h, img_w, _ = self.latest_image.shape
                img_center = img_w / 2
                steer2set = (bbox[0][0] + int(bbox[0][2]/2) - img_center) / img_center * 30
                print(bbox[0][0] + int(bbox[0][2]/2))
                print(steer2set)
                car_movement = 3 if steer2set < 0 else 2
                print("car_movement", car_movement)
                self.brain_ws_client.send_command('SetSpeed,' + str(int(abs(steer2set))))
                self.brain_ws_client.send_command('MoveCar,' + str(car_movement))
            else:
                self.brain_ws_client.send_command('MoveCar,' + str(4))
        else:
            self.show_image_cv2(self.image_label, self.latest_image)

    def handle_audio_stream(self, audio_one_sec):
        if self.car_cmd_mode == 1:  # voice
            self._audio2cmd(audio_one_sec)

    def _audio2cmd(self, audio_array):
        car_movement = np.argmax(self.audio_detector.predict_nparray(audio_array))  # 0~5
        print("predict: ", car_movement)
        if car_movement < 5:
            self.statusbar.showMessage(str(['Forward', 'Backward', 'Left', 'Right', 'Stop'][car_movement]))
            self.brain_ws_client.send_command('MoveCar,' + str(car_movement))
        else:
            self.statusbar.showMessage("Listening...")

    def toggle_lamp(self):
        self.is_lamp_toggle = 1 - self.is_lamp_toggle
        if not self.flash_mode:
            self.brain_ws_client.send_command('Lamp, {}'.format(self.is_lamp_toggle))

    def toggle_flash_mode(self):
        self.flash_mode = self.flash_button.isChecked()

    def set_moving_speed(self):
        # print(self.speed_slider.value())
        self.brain_ws_client.send_command('SetSpeed,' + str(self.speed_slider.value()))

    def on_button(self, button_type):
        move_group = ['forward', 'backward', 'left', 'right', 'stop']
        monitor_group = ['start', 'pause', 'save']
        action_group = ['normal', 'voice', 'clingy']

        if button_type in move_group:
            if self.car_cmd_mode == 0:  # gui
                self.brain_ws_client.send_command('MoveCar,' + str(move_group.index(button_type)))
            else:
                pass
        elif button_type in monitor_group:
            self.brain_ws_client.send_command('Monitor,' + str(monitor_group.index(button_type)))
        elif button_type in action_group:
            car_cmd_mode = action_group.index(button_type)
            self.brain_ws_client.send_command('Mode,' + str(action_group.index(button_type)))
            self.car_cmd_mode = car_cmd_mode
        elif button_type == 'connect_board':
            """
            cam_ip_addr, okPressed = QInputDialog.getText(self, "IP Configuration", "Input", QLineEdit.Normal, "0.0.0.0")
            self.ip_label.setText(cam_ip_addr)
            if not okPressed:
                print('Cancel')
            try:
                self.brain_ws_client.configure_conn(cam_ip_addr)
                self.brain_ws_client.send_command('GuiIP,' + self.ip_addr)
            except Exception as err:
                print("摄像头连接失败！")
            """
            dialog = IPInputDialog()
            if dialog.exec():
                cam_ip, car_ip = dialog.get_inputs()
                self.cam_ip_label.setText(cam_ip)
                self.car_ip_label.setText(car_ip)
                self.brain_ws_client.configure_conn("ws://{}/CarInput".format(car_ip))
                self.cam_ws_client.configure_conn("ws://{}/Camera".format(cam_ip))
                # self.brain_ws_client.send_command('GuiIP,' + self.ip_addr)  # for i2s data transmit
        elif button_type == 'connect':
            # https://blog.csdn.net/acoustic970323/article/details/111990510
            def get_port_name():
                full_name = self.serial_port_combb.currentText()
                # rfind会找到：的位置
                com_name = full_name[0:full_name.rfind('：')]
                return com_name

            self.serial_port = get_port_name()
            self.serial_device.configure_comm(self.serial_port)
        elif button_type == 'capture':
            print('Capture!')
            self.statusbar.showMessage("Start Recording.")
            name = datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f') + '.jpg'
            cv2.imwrite(name, self.latest_image)
        elif button_type == 'recording':
            self.is_recording = 1 - self.is_recording
            if self.is_recording == 1:
                _thread.start_new_thread(self.recording, ())
        else:
            raise Exception("Not support button type")

    # 串口检测
    def port_detect(self):
        # 检测所有存在的串口 将信息存在字典中
        self.port_dict = {}
        # 将其存在列表中
        port_list = list(list_ports.comports())
        # 清除下拉列表中已有的选项
        self.serial_port_combb.clear()
        for port in port_list:
            # 添加到字典里
            self.port_dict["%s" % port[0]] = "%s" % port[1]
            # 添加到下拉列表选项
            self.serial_port_combb.addItem(port[0] + '：' + port[1])
            self.connect_button.setEnabled(True)
        if len(self.port_dict) == 0:
            self.serial_port_combb.addItem('无串口')
            self.connect_button.setEnabled(False)

    def recording(self):
        fps = 20
        width = self.latest_image.shape[1]
        height = self.latest_image.shape[0]
        out_dir_base = os.path.join('.', datetime.now().strftime("%Y-%m-%d-%H-%M") + '.avi')
        out_dir = out_dir_base[:]
        fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
        out = cv2.VideoWriter(out_dir, fourcc, fps, (int(width), int(height)))
        print("Start Recording.")
        self.statusbar.showMessage("Start Recording.")
        last_frame = self.latest_image
        while self.is_recording:
            while (self.latest_image == last_frame).all():
                pass
            last_frame = self.latest_image
            out.write(last_frame)
        out.release()
        print("Finish Recording.")
        self.statusbar.showMessage("Finish Recording.")

    def show_image_qt(self, qlabel, img_path):
        pixmap = QPixmap(img_path)
        pixmap = pixmap.scaled(window.label.size() - QSize(2, 2))
        qlabel.setPixmap(pixmap)
        (f_dir, f_name) = os.path.split(img_path)
        self.statusbar.showMessage("文件名：{}".format(f_name))

    def show_image_cv2(self, qlabel, bgr_img):
        """Qlabel设置cv2图片"""
        show = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        print(np.shape(show))
        show = cv2.resize(show, (int(show.shape[1] / 4) * 4, int(show.shape[0] / 4) * 4))
        showImage = QImage(show.data, show.shape[1], show.shape[0], QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(showImage)
        # pixmap = pixmap.scaled(window.label.size() - QSize(2, 2))
        qlabel.setPixmap(pixmap)

    def get_icon_by_name(icoName):
        """设置图标"""
        file_index = {
            'add': 'add.png',
            'delete': 'delete.png',
            'statistics': 'statistics.png',
            'reddot': 'reddot.png',
            'orangedot': 'orangedot.png',
            'greendot': 'greendot.png',
            'send': 'send.png',
            'zoomall': 'zoomall.png',
            'connect': 'connect.png',
            'continue': 'continue.png',
            'alert': 'alert.png',
            'gear': 'gear.png',
            'generalsettings': 'generalsettings.png',
            'open': 'open.png',
            'loop': 'loop.png',
            'save': 'save.png',
            'stop': 'stop.png',
            'restart': 'continue.png',
            'start': 'start.png',
            'motor': 'motor.png',
            'pause': 'pause.png',
            'disconnect': 'disconnect.png',
            'configure': 'configure.png',
            'pidconfig': 'pidconfig.png',
            'consoletool': 'consoletool.png'
        }
        current_dir = os.path.dirname(__file__)
        icon_path = os.path.join(current_dir, '../resources', file_index[icoName])
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(icon_path), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        return icon


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.window_init()
    window.show()
    sys.exit(app.exec_())
