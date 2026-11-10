"""Dynamixel Protocol 2.0, the few instructions the duck needs. Plain Python on pyserial.

Tested against pi/sim/server.py, which speaks the same bytes. Not yet tested on a real XL330.
"""
import struct
import time

import serial

PING, READ, WRITE, SYNC_READ, SYNC_WRITE, STATUS = 0x01, 0x02, 0x03, 0x82, 0x83, 0x55
BROADCAST = 0xFE
HEADER = bytes((0xFF, 0xFF, 0xFD, 0x00))

# XL330 control table (ROBOTIS e-manual): address, size
BAUD_RATE = (8, 1)
RETURN_DELAY = (9, 1)
OPERATING_MODE = (11, 1)
TORQUE_ENABLE = (64, 1)
POSITION_P = (84, 2)
PROFILE_VELOCITY = (112, 4)
GOAL_POSITION = (116, 4)
PRESENT_VELOCITY = (128, 4)
PRESENT_POSITION = (132, 4)
PRESENT_VOLTAGE = (144, 2)
PRESENT_TEMPERATURE = (146, 1)
MODE_POSITION = 3
TICKS_PER_REV = 4096
VELOCITY_UNIT = 0.229 * 2 * 3.141592653589793 / 60    # rad/s per unit


def _crc_table():
    t = []
    for i in range(256):
        c = i << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x8005) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
        t.append(c)
    return t


_T = _crc_table()


def crc16(data):
    c = 0
    for b in data:
        c = ((c << 8) & 0xFFFF) ^ _T[((c >> 8) ^ b) & 0xFF]
    return c


def stuff(params):
    """FF FF FD in a packet's body gets an FD after it, so it can't look like a header."""
    out, run = bytearray(), 0
    for b in params:
        out.append(b)
        if (run == 0 and b == 0xFF) or (run == 1 and b == 0xFF):
            run += 1
        elif run == 2 and b == 0xFD:
            out.append(0xFD)
            run = 0
        else:
            run = 1 if b == 0xFF else 0
    return bytes(out)


def unstuff(params):
    out, i = bytearray(), 0
    while i < len(params):
        out.append(params[i])
        if params[i:i + 4] == b'\xff\xff\xfd\xfd':
            out += b'\xff\xfd'
            i += 4
        else:
            i += 1
    return bytes(out)


def packet(ident, inst, params=b''):
    body = stuff(bytes((inst,)) + bytes(params))
    head = HEADER + bytes((ident,)) + struct.pack('<H', len(body) + 2)
    pkt = head + body
    return pkt + struct.pack('<H', crc16(pkt))


class DxlError(IOError):
    pass


class Bus:
    """port: a device like /dev/serial0 (the Pi's UART), or socket://127.0.0.1:9000 for the simulated bus."""

    def __init__(self, port, baud=1_000_000, timeout=0.02):
        self.s = serial.serial_for_url(port, baudrate=baud, timeout=timeout)
        self.timeout = timeout

    def close(self):
        self.s.close()

    def _read_exact(self, n):
        out = bytearray()
        end = time.monotonic() + self.timeout
        while len(out) < n:
            b = self.s.read(n - len(out))
            if b:
                out += b
            elif time.monotonic() > end:
                raise DxlError(f'timeout: got {len(out)} of {n} bytes')
        return bytes(out)

    def _status(self):
        """One status packet: (id, error, params)."""
        win = bytearray()
        end = time.monotonic() + self.timeout
        while bytes(win[-4:]) != HEADER:
            b = self.s.read(1)
            if b:
                win += b
            elif time.monotonic() > end:
                raise DxlError('timeout waiting for a status packet')
        rest = self._read_exact(3)
        ident, length = rest[0], struct.unpack('<H', rest[1:3])[0]
        body = self._read_exact(length)
        pkt = HEADER + rest + body
        if struct.unpack('<H', pkt[-2:])[0] != crc16(pkt[:-2]):
            raise DxlError(f'bad CRC from servo {ident}')
        if pkt[7] != STATUS:
            raise DxlError(f'not a status packet: {pkt[7]:#x}')
        data = unstuff(pkt[8:-2])
        return ident, data[0], data[1:]

    def _send(self, pkt):
        self.s.reset_input_buffer()
        self.s.write(pkt)

    def ping(self, ident):
        self._send(packet(ident, PING))
        return self._status()[0] == ident

    def write(self, ident, reg, value):
        addr, size = reg
        self._send(packet(ident, WRITE, struct.pack('<H', addr) + int(value).to_bytes(size, 'little', signed=False)))
        _, err, _ = self._status()
        if err & 0x7F:
            raise DxlError(f'servo {ident} refused write at {addr}: error {err:#x}')

    def read(self, ident, reg, signed=False):
        addr, size = reg
        self._send(packet(ident, READ, struct.pack('<HH', addr, size)))
        _, err, data = self._status()
        if err & 0x7F:
            raise DxlError(f'servo {ident} refused read at {addr}: error {err:#x}')
        return int.from_bytes(data, 'little', signed=signed)

    def sync_write(self, ids, reg, values):
        addr, size = reg
        p = bytearray(struct.pack('<HH', addr, size))
        for i, v in zip(ids, values):
            p.append(i)
            p += int(v).to_bytes(size, 'little', signed=v < 0)
        self.s.write(packet(BROADCAST, SYNC_WRITE, p))     # no reply to a sync write

    def sync_read(self, ids, addr, size):
        """{id: raw bytes}. A servo that doesn't answer raises DxlError: a walking robot can't carry on without it."""
        self._send(packet(BROADCAST, SYNC_READ, struct.pack('<HH', addr, size) + bytes(ids)))
        out = {}
        for _ in ids:
            ident, err, data = self._status()
            if err & 0x7F:
                raise DxlError(f'servo {ident}: error {err:#x}')
            out[ident] = data
        return out
