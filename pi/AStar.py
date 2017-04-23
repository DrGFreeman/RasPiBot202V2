import smbus
import struct
import time

class AStar:
    def __init__(self):
        self.bus = smbus.SMBus(1)

    def read_unpack(self, address, size, format):
        self.bus.write_byte(20, address)
        time.sleep(0.0001)
        byte_list = [self.bus.read_byte(20) for _ in range(size)]
        return struct.unpack(format, bytes(byte_list))

    def write_pack(self, address, format, *data):
        data_array = list(struct.pack(format, *data))
        self.bus.write_i2c_block_data(20, address, data_array)
        time.sleep(0.0001)

    def leds(self, red, yellow, green):
        self.write_pack(0, 'BBB', red, yellow, green)

    def read_buttons(self):
        return self.read_unpack(3, 3, "???")

    def move(self, fwd, turn):
        self.write_pack(6, 'hh', fwd, turn * 1000)

    def read_odometer(self):
        x, y, phi = self.read_unpack(10, 6, 'hhh')
        phi /= 1000
        return x, y, phi

    def read_battery_millivolts(self):
        return self.read_unpack(16, 2, "H")[0]

    def play_notes(self, notes):
        self.write_pack(18, 'B15s', 1, notes.encode("ascii"))
