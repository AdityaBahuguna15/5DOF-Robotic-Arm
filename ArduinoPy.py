import serial
import time

class ArduinoInterface:
    def __init__(self, port='COM5', baud=115200, timeout=1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None

    def connect(self):
        if self.ser is None:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(2.0)
            print(f"[Arduino] connected {self.port} @ {self.baud}")

    def send_angles(self, s1, s2, s3, s4, s5):
        if self.ser is None:
            raise RuntimeError('Serial not connected')
        packet = f"{int(s1)},{int(s2)},{int(s3)},{int(s4)},{int(s5)}\n"
        self.ser.write(packet.encode())

    def read_line(self):
        if self.ser is None:
            return None
        return self.ser.readline().decode(errors='ignore').strip()

    def close(self):
        if self.ser is not None:
            self.ser.close()
            self.ser = None