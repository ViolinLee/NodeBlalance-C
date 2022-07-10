import time
import serial
import asyncio
import logging
import _thread
import threading
import websockets
import websocket
import pyaudio
import socket
import numpy as np
from PySide2 import QtCore, QtWidgets
from serial import SerialException
from queue import Queue
from scipy.io.wavfile import write
from datetime import datetime


class SerialDeviceHandler(QtCore.QThread):
    telemetryDataReceived = QtCore.Signal(str)
    rawDataReceived = QtCore.Signal(str)

    def __init__(self, slot_func):
        super(SerialDeviceHandler, self).__init__()
        self._stop_event = threading.Event()
        self.serial_comm = None
        self.slot_func = slot_func

    def configure_comm(self, port, rate=115200):
        self.serial_comm = serial.Serial(port, rate)
        self.start()

    @staticmethod
    def is_data_received_telemetry(data):
        """data format validation"""
        if data.startswith(b"$$"):
            return True
        else:
            return False

    def handle_received_data(self, data):
        if self.is_data_received_telemetry(data):
            try:
                v = data.lstrip(b'&&').rstrip().split(b'-')
                self.telemetryDataReceived.emit(float(v[0]), float(v[1]), float(v[2]), float(v[2]))
            except ValueError as error:
                logging.error(error, exc_info=True)
                logging.error('data =' + str(data), exc_info=True)
        self.rawDataReceived.emit(data.rstrip())

    def run(self):
        try:
            while not self.stopped():
                if self.serial_comm is not None and self.serial_comm.isOpen():
                    reading = self.serial_comm.readline().decode()
                    self.handle_received_data(reading)
        except SerialException as serialException:
            logging.error(serialException, exc_info=True)

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()


class CamWSClientAsync(QtCore.QObject):
    cam_data_received = QtCore.Signal(bytes)

    def __init__(self, slot_func):
        super().__init__()
        self.url = None
        self.slot_func = slot_func

    def configure_conn(self, url):
        self.url = url
        _thread.start_new_thread(self.start_routine, args=())  # 注意是守护线程

    def start_routine(self):
        # 以URL配置合理为前提
        self.cam_data_received.connect(self.slot_func)
        asyncio.get_event_loop().run_until_complete(self.receive_camera_data())
        asyncio.get_event_loop().run_forever()  # 如果不开启一个子线程，会堵塞在这一句

    async def receive_camera_data(self):
        """要emit信号至更新图片的slot，必须是forever serving"""
        async with websockets.connect(self.url) as ws:
            # await ws.send("begin")
            while True:
                response = await ws.recv()
                self.cam_data_received.emit(response)


class CamWSClientCallback(QtCore.QObject):
    cam_data_received = QtCore.Signal(bytes)

    def __init__(self, slot_func):
        super().__init__()
        self.url = None
        self.slot_func = slot_func

    def configure_conn(self, url):
        self.url = url
        _thread.start_new_thread(self.start_routine, ())

    def on_message(self, wsapp, message):
        self.cam_data_received.emit(message)

    def on_open(self, wsapp):
        print("Opened connection")
        self.cam_data_received.connect(self.slot_func)
        print("slot connected")

    def start_routine(self):
        # websocket.enableTrace(False)
        ws = websocket.WebSocketApp(self.url, on_open=self.on_open, on_message=self.on_message)
        ws.run_forever()


class BrainWSClientAsync:
    def __init__(self):
        self.url = None
        self.command_queue = Queue()

    def configure_conn(self, url):
        self.url = url
        _thread.start_new_thread(self.start_routine, ())

    def start_routine(self):
        new_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(new_loop)
        asyncio.get_event_loop().run_until_complete(self._send_queue_data())
        asyncio.get_event_loop().run_forever()

    async def _send_queue_data(self):
        async with websockets.connect(self.url) as ws:
            while True:
                await ws.send(self.command_queue.get())  # method 'get' is blocking until new data putted

    def send_command(self, command):
        # the maintained queue can avoid frequent establishment and release of websocket links
        self.command_queue.put(command)


class UDPSocketCallback(QtCore.QThread, QtCore.QObject):
    p = pyaudio.PyAudio()
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    CHUNK = 1024
    stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, output=True, frames_per_buffer=CHUNK)

    audio_data_received = QtCore.Signal(bytes)

    def __init__(self, slot_func):
        super(UDPSocketCallback, self).__init__()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.slot_func = slot_func
        self.bytes_buffer = bytes()
        self.audio_data_received.connect(self.slot_func)

    def configure_conn(self, udp_ip, udp_port):
        self.sock.bind((udp_ip, udp_port))

    def run(self):
        try:
            while True:
                data, addr = self.sock.recvfrom(self.CHUNK * 2)  # buffer de 1024 bytes
                self.bytes_buffer = self.bytes_buffer + data
                self.stream.write(data)

                if len(self.bytes_buffer) >= 32768:  # slightly longer than 1s
                    np_data = np.array([self.bytes_buffer[2 * i] + self.bytes_buffer[2 * i + 1] * 256 for i in range(int(len(self.bytes_buffer) / 2))])
                    # print(np.min(np_data), np.max(np_data))
                    scaled = np.interp(np_data, [np.min(np_data), np.max(np_data)], [-32768, 32767]).astype(np.int16)
                    name = datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f')
                    # write('./binary_data/{}.wav'.format(name), 16000, scaled)

                    import wave

                    wf = wave.open("./binary_data/{}.wav".format(name), 'wb')
                    wf.setnchannels(1)
                    wf.setsampwidth(self.p.get_sample_size(pyaudio.paInt16))
                    wf.setframerate(16000)
                    wf.writeframes(self.bytes_buffer)
                    wf.close()

                    self.audio_data_received.emit("./binary_data/{}.wav".format(name))
                    self.bytes_buffer = bytes()

        except KeyboardInterrupt:
            print("Stop streaming")
            self.stream.stop_stream()
            self.stream.close()
            self.p.terminate()


if __name__ == '__main__':
    """
    cam_ws_client = CamWSClientCallback(lambda data: print(data[:50]))
    cam_ws_client.configure_conn(url='ws://localhost:8000')
    
    while True:
        print("Here")
        time.sleep(1)
    """

    brain_ws_client = BrainWSClientAsync()
    brain_ws_client.configure_conn(url='ws://localhost:8000')
    brain_ws_client.send_command('Hello')
    brain_ws_client.send_command('My friend')

    while True:
        print("Here")
        time.sleep(1)
