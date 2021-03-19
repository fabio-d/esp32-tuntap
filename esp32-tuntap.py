import argparse
import ipaddress
import struct
import sys
import subprocess
import threading
import time
from typing import Callable, List

import pytun
import serial


class MCU:
    def __init__(self, serial_port: str, baud_rate: int, skip_init: bool, debug_mcu_output: bool):
        self._serial = serial.Serial(serial_port, 115200, stopbits=serial.STOPBITS_TWO, timeout=None)
        self._baud_rate = baud_rate  # this will be applied later
        self._skip_init = skip_init
        self._debug_mcu_output = debug_mcu_output

        if not self._skip_init:
            self._serial.setDTR(False)
            time.sleep(0.5)
            self._serial.flushInput()
            self._serial.flushOutput()
            self._serial.setDTR(True)

            buffer = b''
            while b'ready to receive configuration' not in buffer:
                c = self._serial.read(1)
                if self._debug_mcu_output:
                    sys.stderr.buffer.write(c)
                    sys.stderr.buffer.flush()
                buffer += c
        else:
            self._serial.flushInput()
            self._serial.flushOutput()

    def tx(self, payload: bytes):
        lenbytes = struct.pack('<H', len(payload))
        data = b'\xAA' + struct.pack('B', 0xFF & -(0xAA + sum(payload) + sum(lenbytes))) + lenbytes + payload
        self._serial.write(data)

    def setup(self, configuration_strings: List[str]):
        if not self._skip_init:
            time.sleep(0.5)
            for s in configuration_strings:
                self._serial.write(s.encode('ascii') + b'\n')
                time.sleep(0.6)
                if self._debug_mcu_output:
                    sys.stderr.buffer.write(self._serial.read_all())
                    sys.stderr.buffer.flush()

        self._serial.baudrate = self._baud_rate

    def rx_loop(self, rx_callback: Callable[[bytes], None]):
        rx_buffer = None
        while True:
            b = self._serial.read(1)
            if b != b'\xAA':
                if self._debug_mcu_output:
                    sys.stderr.buffer.write(b)
                    sys.stderr.buffer.flush()
                continue

            b += self._serial.read(3)
            b += self._serial.read(struct.unpack('<xxH', b)[0])

            if sum(b) & 0xFF == 0:  # validate checksum
                rx_callback(b[4:])


def write_to_pytun_ignore_exceptions(dev: pytun.TunTapDevice, data: bytes):
    try:
        dev.write(data)
    except pytun.Error as e:
        print(e, file=sys.stderr)


def setup_ap(args: argparse.Namespace, mcu: MCU):
    mcu.setup([
        'ssid ' + args.ssid,
        'password ' + args.password if args.password else '',
        'channel ' + str(args.channel),
        'protocols ' + args.protocols,
        'ap ' + str(args.baud)
    ])


def setup_sta(args: argparse.Namespace, mcu: MCU):
    mcu.setup([
        'ssid ' + args.ssid,
        'password ' + args.password if args.password else '',
        'channel ' + str(args.channel),
        'protocols ' + args.protocols,
        'sta {} {} {} {}'.format(
            args.local_address.ip,
            args.gateway_address if args.gateway_address else '0.0.0.0',
            args.local_address.netmask,
            str(args.baud)
        )
    ])


def protocol_string(val: str):
    flags = {'b': '', 'g': '', 'n': '', 'l': ''}
    for c in val:
        if c in flags:
            flags[c] = c
        else:
            raise ValueError(val)
    return ''.join(flags.values())


def main():
    serial_opts = argparse.ArgumentParser(add_help=False)
    serial_opts.add_argument('--serial', default='/dev/ttyUSB0', metavar='PATH',
                             help='serial port path (default: ttyUSB0)')
    serial_opts.add_argument('--baud', default=460800, type=int,
                             help='serial port baud rate (default: 460800)')
    serial_opts.add_argument('--skip-init', action='store_true',
                             help='skip MCU reset and initialization')
    serial_opts.add_argument('--debug-mcu-output', action='store_true',
                             help='print MCU output that does not look like a packet to stderr')

    wireless_opts = argparse.ArgumentParser(add_help=False)
    wireless_opts.add_argument('--ssid', required=True,
                               help='set network name')
    wireless_opts.add_argument('--password',
                               help='set network password (default: unprotected network)')
    wireless_opts.add_argument('--channel', type=int, default=1,
                               help='set wireless channel (default: 1)')
    wireless_opts.add_argument('--protocols', type=protocol_string, default='bgn',
                               help='enabled protocols ("b", "g", "n", and/or "l" - default: bgn)')

    tuntap_opts = argparse.ArgumentParser(add_help=False)
    tuntap_opts.add_argument('--interface', default='', metavar='NAME',
                             help='force interface name')
    tuntap_opts.add_argument('--up', action='store_true',
                             help='automatically activate interface')

    parser = argparse.ArgumentParser(prog='esp32-tuntap.py',
                                     description='esp32-tuntap Linux agent: use an ESP32 board as wireless dongle')
    subparsers = parser.add_subparsers(metavar='COMMAND', required=True)

    ap = subparsers.add_parser('ap', parents=[serial_opts, wireless_opts, tuntap_opts],
                               help='Access Point (AP) exposed as a TAP interface')
    ap.add_argument('--add-to-bridge', metavar='BRIDGE_NAME',
                    help='add the interface to the specified bridge')
    ap.add_argument('--local-address', type=ipaddress.IPv4Interface, metavar='ADDRESS/MASK',
                    help='set local IP address and network mask')
    ap.add_argument('--gateway-address', type=ipaddress.IPv4Address, metavar='ADDRESS',
                    help='set default gateway IP address')
    ap.set_defaults(interface_flags=pytun.IFF_TAP, setup_func=setup_ap)

    sta = subparsers.add_parser('sta', parents=[serial_opts, wireless_opts, tuntap_opts],
                                help='Client (STA) exposed as a TUN interface')
    sta.add_argument('--local-address', required=True, type=ipaddress.IPv4Interface, metavar='ADDRESS/MASK',
                     help='set local IP address and network mask')
    sta.add_argument('--gateway-address', type=ipaddress.IPv4Address, metavar='ADDRESS',
                     help='set default gateway IP address')
    sta.set_defaults(interface_flags=pytun.IFF_TUN, setup_func=setup_sta)

    args = parser.parse_args()
    if args.gateway_address is not None and (args.local_address is None or not args.up):
        parser.error('--gateway-address requires --local-address and --up too')

    mcu = MCU(args.serial, args.baud, args.skip_init, args.debug_mcu_output)

    dev = pytun.TunTapDevice(args.interface, flags=args.interface_flags | pytun.IFF_NO_PI)
    if args.up:
        dev.up()
    if args.local_address is not None:
        dev.addr = str(args.local_address.ip)
        dev.netmask = str(args.local_address.netmask)
        if args.gateway_address is not None:
            subprocess.check_call('ip route add default via ' + str(args.gateway_address), shell=True)
    elif args.add_to_bridge is not None:
        subprocess.check_call('ip link set ' + dev.name + ' master ' + str(args.add_to_bridge), shell=True)

    args.setup_func(args, mcu)

    print('Ready!', file=sys.stderr)

    rx = threading.Thread(target=mcu.rx_loop, args=[lambda d: write_to_pytun_ignore_exceptions(dev, d)], daemon=True)
    rx.start()

    while True:
        mcu.tx(dev.read(1996))


if __name__ == '__main__':
    main()
