import machine
import time
from micropython import const


class SCD4X:
    """
    Based on https://github.com/adafruit/Adafruit_CircuitPython_SCD4X
    Copyright (c) 2021 ladyada for Adafruit Industries
    MIT License
    """

    DEFAULT_ADDRESS = 0x62
    DATA_READY = const(0xE4B8)
    STOP_PERIODIC_MEASUREMENT = const(0x3F86)
    START_PERIODIC_MEASUREMENT = const(0x21B1)
    READ_MEASUREMENT = const(0xEC05)

    def __init__(self, i2c_bus, address=DEFAULT_ADDRESS):
        self.i2c = i2c_bus
        self.address = address
        self._buffer = bytearray(18)
        self._cmd = bytearray(2)
        self._crc_buffer = bytearray(2)

        # cached readings
        self._temperature = None
        self._relative_humidity = None
        self._co2 = None

        self.stop_periodic_measurement()

    @property
    def co2(self):
        """Returns the CO2 concentration in PPM (parts per million)
        .. note::
            Between measurements, the most recent reading will be cached and returned.
        """
        if self.data_ready:
            self._read_data()
        return self._co2

    @property
    def temperature(self):
        """Returns the current temperature in degrees Celsius
        .. note::
            Between measurements, the most recent reading will be cached and returned.
        """
        if self.data_ready:
            self._read_data()
        return self._temperature

    @property
    def relative_humidity(self):
        """Returns the current relative humidity in %rH.
        .. note::
            Between measurements, the most recent reading will be cached and returned.
        """
        if self.data_ready:
            self._read_data()
        return self._relative_humidity

    def _read_data(self):
        """Reads the temp/hum/co2 from the sensor and caches it"""
        self._send_command(self.READ_MEASUREMENT, cmd_delay=0.001)
        self._read_reply(self._buffer, 9)
        self._co2 = (self._buffer[0] << 8) | self._buffer[1]
        temp = (self._buffer[3] << 8) | self._buffer[4]
        self._temperature = -45 + 175 * (temp / 2 ** 16)
        humi = (self._buffer[6] << 8) | self._buffer[7]
        self._relative_humidity = 100 * (humi / 2 ** 16)

    @property
    def data_ready(self):
        """Check the sensor to see if new data is available"""
        self._send_command(self.DATA_READY, cmd_delay=0.001)
        self._read_reply(self._buffer, 3)
        return not ((self._buffer[0] & 0x03 == 0) and (self._buffer[1] == 0))

    def stop_periodic_measurement(self):
        """Stop measurement mode"""
        self._send_command(self.STOP_PERIODIC_MEASUREMENT, cmd_delay=0.5)

    def start_periodic_measurement(self):
        """Put sensor into working mode, about 5s per measurement"""
        self._send_command(self.START_PERIODIC_MEASUREMENT, cmd_delay=0.01)

    def _send_command(self, cmd, cmd_delay=0.0):
        self._cmd[0] = (cmd >> 8) & 0xFF
        self._cmd[1] = cmd & 0xFF
        self.i2c.writeto(self.address, self._cmd)
        time.sleep(cmd_delay)

    def _read_reply(self, buff, num):
        self.i2c.readfrom_into(self.address, buff, num)
        self._check_buffer_crc(self._buffer[0:num])

    def _check_buffer_crc(self, buf):
        for i in range(0, len(buf), 3):
            self._crc_buffer[0] = buf[i]
            self._crc_buffer[1] = buf[i + 1]
            if self._crc8(self._crc_buffer) != buf[i + 2]:
                raise RuntimeError("CRC check failed while reading data")
        return True

    @staticmethod
    def _crc8(buffer):
        crc = 0xFF
        for byte in buffer:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
        return crc & 0xFF  # return the bottom 8 bits


class AQM1602:
    """
    Based on https://akizukidenshi.com/catalog/g/gP-08779/
    """

    DEFAULT_ADDRESS = 0x3E

    def __init__(self, i2c_bus, address=DEFAULT_ADDRESS):
        self.i2c = i2c_bus
        self.address = address
        time.sleep_ms(100)
        self.write_cmd(0x38)
        time.sleep_ms(20)
        self.write_cmd(0x39)
        time.sleep_ms(20)
        self.write_cmd(0x14)
        time.sleep_ms(20)
        self.write_cmd(0x73)
        time.sleep_ms(20)
        self.write_cmd(0x56)
        time.sleep_ms(20)
        self.write_cmd(0x6C)
        time.sleep_ms(20)
        self.write_cmd(0x38)
        time.sleep_ms(20)
        self.write_cmd(0x01)
        time.sleep_ms(20)
        self.write_cmd(0x0C)
        time.sleep_ms(20)

    def write_data(self, data):
        self.i2c.writeto_mem(self.address, 0x40, bytes([data & 0xFF]), addrsize=8)
        time.sleep_ms(1)

    def write_cmd(self, cmd):
        self.i2c.writeto_mem(self.address, 0x00, bytes([cmd & 0xFF]), addrsize=8)
        time.sleep_ms(1)

    def print(self, line_no, lin):
        buf = bytearray(lin)
        if len(buf) <= 0:
            return
        if len(buf) > 16:
            buf = buf[0:16]
        if line_no == 0:
            self.write_cmd(0x01)
            self.write_cmd(0x80)
        else:
            self.write_cmd(0x02)
            self.write_cmd(0xC0)
        for idx in range(0, len(buf)):
            self.write_data(buf[idx])


if __name__ == "__main__":
    led = machine.Pin(25, machine.Pin.OUT)
    uart = machine.UART(1, 9600, tx=machine.Pin(4), rx=machine.Pin(5))

    # I2C0: SCD4X
    i2c0 = machine.I2C(0, sda=machine.Pin(0), scl=machine.Pin(1), freq=100000)
    print("i2c0 scan result:", i2c0.scan())
    scd4x = SCD4X(i2c0)
    scd4x.start_periodic_measurement()

    # I2C1: LCD
    i2c1 = machine.I2C(1, sda=machine.Pin(2), scl=machine.Pin(3), freq=100000)
    print("i2c1 scan result:", i2c1.scan())
    lcd = AQM1602(i2c1)

    seq = 0
    print("seq,co2,temperature,humidity")
    lcd.print(0, "SCD4X CO2 Sensor")
    lcd.print(1, "Initializing...")

    while True:
        time.sleep(5)
        seq = seq + 1
        co2_ppm = scd4x.co2
        temp_deg = scd4x.temperature
        humidity_percent = scd4x.relative_humidity
        print(seq, co2_ppm, temp_deg, humidity_percent, sep=",")
        if co2_ppm >= 2000:
            led.on()
        else:
            led.off()
        lcd.print(0, "%dppm" % co2_ppm)
        lcd.print(1, "%.1f%cC %.1f%%" % (temp_deg, 13, humidity_percent))
