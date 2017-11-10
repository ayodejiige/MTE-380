import serial
import time
import logging

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)

class SerialCom(object):
    """docstring for SerialCom"""
    START = 0xFE
    FORWARD = 0x01
    UP = 0x02
    DOWN = 0x03
    LEFT = 0x04
    RIGHT = 0x05
    MIN_MAG = 0x00
    MAX_MAG = 0xFA
    END = 0xFF
    def __init__(self, port="COM4", baudrate=9600):
        super(SerialCom, self).__init__()
        self._port = port
        self._baudrate = baudrate
        self._log = logging.getLogger(self.__class__.__name__)
        try:
            self._com = serial.Serial(self._port, baudrate=self._baudrate)
        except Exception as e:
            self._log.error(e)

    def send_direction(self, direction, magnitude):
        if ((type(direction) != int) & (type(magnitude) != int)):
            self._log.error("Direction or magnitude wrong type")
            return
        if((direction > self.RIGHT) | (direction < self.FORWARD)):
            self._log.error("Wrong directions")
            return
        if((magnitude < self.MIN_MAG) | (magnitude > self.MAX_MAG)):
            self._log.error("Wrong magnitude")
            return
        command = bytearray([direction, magnitude])
        self._com.write(command)
        self._log.info("Sending %d %d", int(command[0]), int(command[1]))

    def send(self, data):
        data = [self.START] + data + [self.END]
        command = bytearray(data)
        self._com.write(command)
        command_print = " ".join(str(x) for x in data)
        self._log.info("Sending %s", command_print)
    
    def recv(self, n_bytes):
        data = self._com.read(n_bytes)
        return data

def main():
    com = SerialCom("COM7")
    while True:
        print com.recv(10)

if __name__ == '__main__':
    main()
