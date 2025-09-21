import serial
import RPi.GPIO as GPIO
import time

TXE = 18
RXE = 4


def gpio_init():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup((TXE, RXE), GPIO.OUT)


class ServoDriver(object):
    def __init__(self):
        gpio_init()
        self.delay = {3: 0.00042, 5: 0.00055, 7: 0.0008}
        self.port = serial.Serial('/dev/ttyS0', 115200, timeout=0.2)

    def _dir(self, write=True):
        GPIO.output(TXE, write)
        GPIO.output(RXE, not write)

    def _write(self, node, cmd, param1=None, param2=None):
        data = [0x55, 0x55, node & 0xff, 0, cmd & 0xff]
        data += [param1 & 0xff, (param1 >> 8) & 0xff] if param1 else []
        data += [param2 & 0xff, (param2 >> 8) & 0xff] if param2 else []
        data[3] = len(data) - 2
        data.append(~sum(data[2:]) & 0xff)
        # print('%02X ' * len(data) % tuple(data), self.delay[data[3]])
        self._dir(True)
        self.port.write(data)
        time.sleep(self.delay[data[3]])
        self._dir(False)

    def _write_byte(self, node, cmd, value):
        data = [0x55, 0x55, node & 0xff, 4, cmd & 0xff, value & 0xff, ~(node+4+cmd+value) & 0xff]
        self._dir(True)
        self.port.write(data)
        time.sleep(0.00055)
        self._dir(False)

    def _read(self, node, cmd, redo=3):
        self.port.flush()
        self._write(node, cmd)
        time.sleep(0.1)
        rel = self.port.read_all()
        if len(rel) < 7:
            print(rel)
            return self._read(node, cmd, redo - 1) if redo else False
        if rel[0] != 0x55 or rel[1] != 0x55:
            print('head error')
            print('%02X ' * len(rel) % tuple(rel))
            return self._read(node, cmd, redo - 1) if redo else False
        if ~sum(rel[2: rel[3] + 2]) & 0xff != rel[rel[3] + 2]:
            print('checksum error')
            print('%02X ' * len(rel) % tuple(rel))
            return self._read(node, cmd, redo - 1) if redo else False
        rel = rel[5:rel[3] + 2]
        # print('%02X ' * len(rel) % tuple(rel))
        return rel

    def move_time_write(self, node, pos, tim):
        self._write(node, 1, pos, tim)

    def move_time_read(self, node):
        rel = self._read(node, 2)
        # print('%02X ' * len(rel) % tuple(rel))
        if not rel or len(rel) != 4:
            print('move time read error')
            return None
        return int.from_bytes(rel[:2], 'little'), int.from_bytes(rel[2:], 'little')

    def move_time_wait_write(self, node, pos, tim):
        self._write(node, 7, pos, tim)

    def move_time_wait_read(self, node, redo=3):
        rel = self._read(node, 8, redo)
        # print('%02X ' * len(rel) % tuple(rel))
        if not rel or len(rel) != 4:
            print('move time read error')
            return None
        return int.from_bytes(rel[:2], 'little'), int.from_bytes(rel[2:], 'little')

    def move_start(self, node):
        self._write(node, 11)

    def move_stop(self, node):
        self._read(node, 12)

    def id_write(self, node, value):
        self._write_byte(node, 13, value)

    def id_read(self, node):
        rel = self._read(node, 14)
        if not rel or len(rel) != 1:
            print('id read error')
            return None
        return rel[0]

    def angle_offset_adjust(self, node, value):
        value = value if value > 0 else 256+value
        self._write_byte(node, 17, value&0xff)

    def angle_offset_write(self, node):
        self._write(node, 18)

    def angle_offset_read(self, node):
        rel = self._read(node, 19)
        if not rel or len(rel) != 1:
            print('angle offset read error')
            return None
        return rel[0]

    def angle_limit_write(self, node, min_angle, max_angle):
        self._write(node, 20, min_angle, max_angle)

    def angle_limit_read(self, node, redo=3):
        rel = self._read(node, 21, redo)
        # print('%02X ' * len(rel) % tuple(rel))
        if not rel or len(rel) != 4:
            print('angle limit read error')
            return None
        return int.from_bytes(rel[:2], 'little'), int.from_bytes(rel[2:], 'little')

    def vin_limit_write(self, node, min_vin, max_vin):
        self._write(node, 22, min_vin, max_vin)

    def vin_limit_read(self, node, redo=3):
        rel = self._read(node, 23, redo)
        # print('%02X ' * len(rel) % tuple(rel))
        if not rel or len(rel) != 4:
            print('vin limit read error')
            return None
        return int.from_bytes(rel[:2], 'little'), int.from_bytes(rel[2:], 'little')

    def temp_max_limit_write(self, node, temp):
        self._write_byte(node, 24, temp)

    def temp_max_limit_read(self, node):
        rel = self._read(node, 25)
        if not rel or len(rel) != 1:
            print('temp max limit read error')
            return None
        return rel[0]

    def temp_read(self, node):
        rel = self._read(node, 26)
        if not rel or len(rel) != 1:
            print('temp read error')
            return None
        return rel[0]

    def vin_read(self, node):
        rel = self._read(node, 27)
        if not rel or len(rel) != 2:
            print('vin read error')
            return None
        return int.from_bytes(rel[:2], 'little'), int.from_bytes(rel[2:], 'little')

    def pos_read(self, node):
        rel = self._read(node, 28)
        if not rel or len(rel) != 2:
            print('pos read error')
            return None
        return int.from_bytes(rel[:2], 'little'), int.from_bytes(rel[2:], 'little')

    def mode_write(self, node, mode, speed):
        self._write(node, 29, mode, speed)

    def mode_read(self, node, redo=3):
        rel = self._read(node, 30, redo)
        # print('%02X ' * len(rel) % tuple(rel))
        if not rel or len(rel) != 4:
            print('or motor mode read error')
            return None
        return int.from_bytes(rel[:2], 'little'), int.from_bytes(rel[2:], 'little')

    def load_or_unload_write(self, node, value):
        self._write_byte(node, 31, value)

    def load_or_unload_read(self, node):
        rel = self._read(node, 32)
        if not rel or len(rel) != 1:
            print('load or unload read error')
            return None
        return rel[0]

    def led_ctrl_write(self, node, value):
        self._write_byte(node, 33, value)

    def led_ctrl_read(self, node):
        rel = self._read(node, 34)
        if not rel or len(rel) != 1:
            print('led ctrl read error')
            return None
        return rel[0]

    def led_error_write(self, node, value):
        self._write_byte(node, 35, value)

    def led_error_read(self, node):
        rel = self._read(node, 36)
        if not rel or len(rel) != 1:
            print('led error read error')
            return None
        return rel[0]

if __name__ == '__main__':
    demo = ServoDriver()
    demo.id_write(1,7)
    demo.move_time_write(7,400,200)
       

