#!/usr/bin/env python3
"""
Read values from a local SolarEdge inverter using the SunSpec open
protocol and Modbus TCP.

This example connects using Modbus TCP to the specified IP and port,
reads registers and outputs values.

This is still an example and a work in progress.

Example usage::

    $ ./pe32sunspecpy_pub.py 192.168.x.y 1502

    C_SunSpec_ID      1400204883
    C_SunSpec_DID     1
    C_SunSpec_Length  65
    C_Manufacturer    SolarEdge
    C_Model           SE5000H-RW000BNN4
    C_Version         0004.0011.0030
    C_SerialNumber    12345678
    C_DeviceAddress   1

    C_SunSpec_DID     101
    C_SunSpec_Length  50
    I_AC_Power        1980.7
    I_AC_Frequency    50.007
    I_AC_Energy_WH    2826662
    I_DC_Power        2010.8
    I_Temp_Sink       50.68
    I_Status          SunspecInverterStatus.I_STATUS_MPPT
    I_Status_Vendor   0


Enabling Modbus TCP on the SolarEdge first?
-------------------------------------------

You may need to enable Modbus TCP first. To do this, you do the WiFi
setup in the SolarEdge app (or SetApp). Then use a browser to go to
172.168.0.1 and surf to "Commissioning" -> "Site communication" ->
"Modbus TCP port" -> "Enable".

http://172.16.0.1/#/commissioning/communication/modbus-tcp

(See the QR-code on your SolarEdge inverter to read the password to
connect to the network without the app.)
"""
import asyncio
import os
import struct
import sys
import time
from collections import OrderedDict
from decimal import Decimal
from enum import Enum

from asyncio_mqtt import Client as MqttClient

__version__ = 'pe32sunspecpy_pub-FIXME'


class DecimalWithUnit(Decimal):
    @classmethod
    def with_unit(cls, value, unit):
        ret = cls(value)
        ret.unit = unit
        return ret

    def __str__(self, format_spec=''):
        return '{} {}'.format(super().__str__(), self.unit)
    __format__ = __str__


class RType:
    """
    Various register data conversions, operating on a callable that
    takes an offset and an octet count.
    """
    class CAST:
        "Cast U16 to the specified type"
        def __init__(self, type_):
            self._type = type_

        def __call__(self, getter, offset=0):
            return self._type(struct.unpack('>H', getter(offset, 2))[0])

    class BSTR:
        "Binary string (trailing spaces and NULs)"
        def __init__(self, len):
            self.len = len

        def __call__(self, getter, offset=0):
            return getter(offset, self.len)

    class STR(BSTR):
        "String (stripped of trailing blanks)"
        def __call__(self, getter, offset=0):
            return (
                super().__call__(getter, offset).rstrip(b'\x00\x20')
                .decode('utf-8'))

    @staticmethod
    def I16(getter, offset=0):
        return struct.unpack('>h', getter(offset, 2))[0]

    @staticmethod
    def U16(getter, offset=0):
        return struct.unpack('>H', getter(offset, 2))[0]

    @staticmethod
    def I32(getter, offset=0):
        return struct.unpack('>i', getter(offset, 4))[0]

    @staticmethod
    def U32(getter, offset=0):
        return struct.unpack('>I', getter(offset, 4))[0]


class SunspecInverterStatus(Enum):
    I_STATUS_OFF = 1            # Off
    I_STATUS_SLEEPING = 2       # Sleeping (auto-shutdown) - Night mode
    I_STATUS_STARTING = 3       # Grid Monitoring/wake-up
    I_STATUS_MPPT = 4           # Inverter is ON and producing power
    I_STATUS_THROTTLED = 5      # Production (curtailed)
    I_STATUS_SHUTTING_DOWN = 6  # Shutting down
    I_STATUS_FAULT = 7          # Fault
    I_STATUS_STANDBY = 8        # Maintenance/setup


# SolarEdge SE5000H and probably others
# https://www.solaredge.com/sites/default/files/
#   sunspec-implementation-technical-note.pdf
SUNSPEC_COMMON_MODEL_REGISTER_MAPPINGS = (
    (40000, 'C_SunSpec_ID', RType.U32),         # 0x53756e53 "SunS"
    (40002, 'C_SunSpec_DID', RType.U16),        # 1
    (40003, 'C_SunSpec_Length', RType.U16),     # 65 + 40003 + 1 == 40069
    (40004, 'C_Manufacturer', RType.STR(32)),   # "SolarEdge"
    (40020, 'C_Model', RType.STR(32)),          # "SE5000H-RW000BNN4"
    (40044, 'C_Version', RType.STR(16)),        # "0004.0011.0030"
    (40052, 'C_SerialNumber', RType.STR(16)),   # "12345678"
    (40068, 'C_DeviceAddress', RType.U16),      # 1
    (40069,),                                   # EOF
)

# SolarEdge SE5000H and probably others
# https://www.solaredge.com/sites/default/files/
#   sunspec-implementation-technical-note.pdf
SUNSPEC_INVERTER_MODEL_REGISTER_MAPPINGS = (
    # 40069: [101] = single phase, 102 = split phase, 103 = threephase
    (40069, 'C_SunSpec_DID', RType.U16),        # 101 (= single phase)
    (40070, 'C_SunSpec_Length', RType.U16),     # 50 + 40070 + 1 == 40121
    (40083, 'I_AC_Power', RType.U16, 'W'),      # AC Power value [Watt]
    (40084, 'I_AC_Power_SF', RType.I16),        # AC Power scale (exp)
    (40085, 'I_AC_Frequency', RType.U16, 'Hz'),  # AC Frequency value [Hertz]
    (40086, 'I_AC_Frequency_SF', RType.I16),    # AC Frequency scale (exp)
    (40093, 'I_AC_Energy_WH', RType.U32, 'Wh'),  # AC Lifetime Energy prod [WH]
    (40095, 'I_AC_Energy_WH_SF', RType.I16),    # AC Lifetime Energe sc. (exp)
    (40100, 'I_DC_Power', RType.U16, 'W'),      # DC Power value [Watt]
    (40101, 'I_DC_Power_SF', RType.I16),        # DC Power scale (exp)
    (40103, 'I_Temp_Sink', RType.I16, '°C'),    # Heat Sink temp [C]
    (40106, 'I_Temp_Sink_SF', RType.I16),       # Heat Sink scale (exp)
    (40107, 'I_Status', RType.CAST(SunspecInverterStatus)),  # Oper. State
    (40108, 'I_Status_Vendor', RType.U16),      # Vendor-defined state
    (40121,),                                   # EOF
)

# SolarEdge SE5000H example to fetch only I_AC_Power
SUNSPEC_INVERTER_MODEL_ONLY_AC_POWER = (
    (40083, 'I_AC_Power', RType.U16, 'W'),      # AC Power value [Watt]
    (40084, 'I_AC_Power_SF', RType.I16),        # AC Power scale (exp)
    (40085,)
)


class Registers:
    """
    One or more register values, initialized with binary data

    Because half the time we don't want 16-bit integers, we'll use this
    to hold the raw octets and return values from there instead.

    Use [0:2] style slices to get the canonical unsigned 16-bits shorts.

    Use packed() to access the raw data, using 16-bit offsets and
    (8-bit) octet counts. For example:

        # Get 32-bits integer (4 bytes) from the 1st (0-based) and 2nd
        # 16-bit registers.
        struct.unpack('>I', registers.packed(1, 4))
    """
    def __init__(self, register_octets):
        self._packed = register_octets

    def packed(self, offset, octets):
        start, stop = offset * 2, offset * 2 + octets
        return self._packed[start:stop]

    def __getitem__(self, slice_):
        if isinstance(slice_, int):
            start, stop = slice_, slice_ + 1
            return self._get_u16_list(start, stop)[0]

        if slice_.step is not None:
            raise NotImplementedError

        start, stop = slice_.start, slice_.stop
        if start is None:
            start = 0
        if stop is None:
            stop = len(self._packed) // 2

        if start < 0 or stop < 0:
            raise NotImplementedError

        return self._get_u16_list(start, stop)

    def _get_u16_list(self, start, stop):
        len_ = stop - start
        values = struct.unpack(
            '>{}'.format('H' * len_), self._packed[(start * 2):(stop * 2)])
        return values


class RegisterIface:
    """
    Wrapper around Registers, giving it an offset and type conversion

    Example usage::

        rs = Registers(b'\\x12\\x34\\x56\\x78')
        rs.packed(0, 2) == b'\\x12\\x34'

        rif = RegisterIface(40000, rs)
        rif.get(40000, RType.U32) == 0x12345678
    """
    def __init__(self, offset, registers):
        self._off = offset
        self._registers = registers

    def get(self, offset, type_):
        """
        Get datatype type_ from registers at absolute offset

        type_ must be a callable that takes a callable and an offset.

        See: RType class.
        """
        return type_(self._registers.packed, offset - self._off)

    def mapping2dict(self, mapping):
        """
        Take a list of (offset, name, type) tuples, return an OrderedDict

        Example usage::

            MAPPING = [
                (40000, 'C_SunSpec_ID', RType.U32),
                ...
                (40004, 'C_Manufacturer', RType.STR(32)),
                ...
                (40069,),
            ]
            r.mapping2dict(MAPPING)
            # ^- returns a dict with key 'C_SunSpec_ID' and the unsigned
            #    int32 taken from offset 40000.
        """
        ret = OrderedDict()
        assert mapping and len(mapping[-1]) == 1, 'expected END marker'
        for item in mapping[0:-1]:  # don't fetch END marker
            offset, name, type_, args = item[0], item[1], item[2], item[3:]
            value = self.get(offset, type_)

            if args:
                assert len(args) == 1, item
                assert isinstance(value, (Decimal, int)), (type(value), value)
                value = DecimalWithUnit.with_unit(value, args[0])

            ret[name] = value

        return ret


class SunspecRegs(RegisterIface):
    def mapping2dict(self, mapping):
        """
        Extend RegisterIface to automatically handle "_SF" (scale factor) keys

        All keys ending with "_SF" are applied and removed from the dictionary.

        Instead of:

            {'I_AC_Frequency': 50015, 'I_AC_Frequency_SF': -3, ...}

        It returns:

            {'I_AC_Frequency': Decimal('50.015'), ...}
        """
        ret = super().mapping2dict(mapping)
        sf_keys = [key for key in ret.keys() if key.endswith('_SF')]
        for sf_key in sf_keys:
            key = sf_key[0:-3]
            assert key in ret, (key, ret)
            exp = Decimal(10) ** ret[sf_key]  # *10^x (E-1, E01, ...)
            if isinstance(ret[key], DecimalWithUnit):
                value = DecimalWithUnit.with_unit(
                    ret[key] * exp, ret[key].unit)
            else:
                value = Decimal(ret[key]) * exp
            ret[key] = value
            del ret[sf_key]
        return ret


class ModbusFrame:
    """
    Modbus request/response packet

    Example transcript:

      >> 0001 0000 0006 0103 9c40 0045
      - transaction 1, protocol 0, 6 bytes, unit 1, function 3
      - from offset 0x9c40 get 0x45 registers
      << 0001 0000 008d 0103 8a 5375 6e53 0001 ...
      - transaction 1, protocol 0, 0x8d bytes, unit 1, function 3
      - get 0x8a bytes of data
      - all registers as uint16
    """
    @classmethod
    def read_multiple_registers(cls, transid, unitid, offset, count):
        return ModbusFrame(
            transid, unitid, 0x3, bytes([
                offset >> 8, offset & 0xff, count >> 8, count & 0xff]))

    @classmethod
    def unpack(cls, bytes_):
        assert len(bytes_) >= 8, bytes_
        transid = (bytes_[0] << 8 | bytes_[0])
        assert bytes_[2] == bytes_[3] == 0, bytes_
        datalen = (bytes_[4] << 8 | bytes_[5])
        assert datalen >= 2, bytes_
        assert len(bytes_) >= (datalen + 6), bytes_
        unitid = bytes_[6]
        funccode = bytes_[7]
        data = bytes_[8:]
        return ModbusFrame(transid, unitid, funccode, data)

    def __init__(self, transid, unitid, funccode, data):
        assert 0 <= transid <= 65535, transid
        self.transid = transid
        self.unitid = unitid
        # 3 = Read Multiple Holding Registers
        # https://en.wikipedia.org/wiki/Modbus#
        #   Available_function/command_codes
        self.funccode = funccode
        self.data = data

    def data_as_registers(self):
        """
        Decode the data octets as big endian 16-bit unsigned ints (registers).

        Feels a bit superfluous perhaps, as we may treat these as other
        data types later on, but let's stick to the protocol.
        """
        assert self.data, self.data
        assert (self.data[0] % 2) == 0, self.data
        hlen = self.data[0] >> 1
        assert len(self.data) >= (hlen + 1), self.data
        # manual = tuple((h << 8 | l) for h, l in zip(
        #     self.data[1::2], self.data[2::2]))
        # faster = struct.unpack('>{}'.format('H' * hlen), self.data[1:])
        lazy = Registers(self.data[1:(1 + hlen * 2)])
        # immediate = lazy[:]
        return lazy

    def pack(self):
        # https://en.wikipedia.org/wiki/Modbus#
        #   Modbus_TCP_frame_format_(primarily_used_on_Ethernet_networks)
        datalen = (1 + 1 + len(self.data))  # unit+funccode+data
        return bytes([
            self.transid >> 8,      # 0: transaction id - copied by server
            self.transid & 0xff,    # 1: transaction id - copied by server
            0,                      # 2: protocol id - 0
            0,                      # 3: protocol id - 0
            datalen >> 8,           # 4: data length
            datalen & 0xff,         # 5: data length
            self.unitid,            # 6: unit identifier ('slave address')
            self.funccode,          # 7: MODBUS function code
        ]) + self.data


class SunspecModbusTcpAsyncio:
    def __init__(self, reader, writer):
        self.reader, self.writer = reader, writer
        self.transid = 0

    async def get_from_mapping(self, mapping, unit=1):
        first_reg = mapping[0][0]
        eof_reg = mapping[-1][0]
        count = eof_reg - first_reg

        self.transid += 1
        request = ModbusFrame.read_multiple_registers(
            self.transid, unit, first_reg, count)
        self.writer.write(request.pack())

        bytes_ = await self.reader.read(4096)
        response = ModbusFrame.unpack(bytes_)
        registers = response.data_as_registers()

        sunspecregs = SunspecRegs(first_reg, registers)
        return sunspecregs.mapping2dict(mapping)


class Pe32SunspecPublisher:
    def __init__(self):
        self._mqtt_broker = os.environ.get(
            'PE32SUNSPEC_BROKER', 'test.mosquitto.org')
        self._mqtt_topic = os.environ.get(
            'PE32SUNSPEC_TOPIC', 'myhome/infra/solar/xwwwform')
        self._mqttc = None
        self._guid = os.environ.get(
            'PE32SUNSPEC_GUID', 'EUI48:11:22:33:44:55:66')

    def open(self):
        # Unfortunately this does use a thread for keepalives. Oh well.
        # As long as it's implemented correctly, I guess we can live
        # with it.
        self._mqttc = MqttClient(self._mqtt_broker)
        return self._mqttc

    async def publish(self, kv):
        #log.debug(f'publish: {kv}')
        print(f'publish: {kv}')

        tm = int(time.time())
        mqtt_string = (
            f'device_id={self._guid}&'
            f's_act_energy_wh={int(kv["I_AC_Energy_WH"])}&'
            f's_inst_power_w={int(kv["I_AC_Power"])}&'
            f's_temperature={float(kv["I_Temp_Sink"])}&'
            f's_status={kv["I_Status"]}:{kv["I_Status_Vendor"]}&'
            f'dbg_uptime={tm}&'
            f'dbg_version={__version__}').encode('ascii')

        await self._mqttc.publish(self._mqtt_topic, payload=mqtt_string)

        print(f'Published: {mqtt_string}')


async def mainloop(host, port):

    publisher = Pe32SunspecPublisher()
    async with publisher.open():
        while True:
            reader, writer = await asyncio.open_connection(host, port)
            c = SunspecModbusTcpAsyncio(reader, writer)

            # d = await c.get_from_mapping(SUNSPEC_COMMON_MODEL_REGISTER_MAPPINGS)
            # assert d['C_SunSpec_ID'] == 0x53756e53, 'C_SunSpec_ID != "SunS"'
            # for key, value in d.items():
            #     print('{:16}  {}'.format(key, value))
            # print()

            d2 = await c.get_from_mapping(SUNSPEC_INVERTER_MODEL_REGISTER_MAPPINGS)
            writer.close()
            for key, value in d2.items():
                print('{:16}  {}'.format(key, value))
            print()

            await publisher.publish(d2)
            await asyncio.sleep(60)


async def oneshot(host, port):
    reader, writer = await asyncio.open_connection(host, port)
    c = SunspecModbusTcpAsyncio(reader, writer)

    d = await c.get_from_mapping(SUNSPEC_COMMON_MODEL_REGISTER_MAPPINGS)
    assert d['C_SunSpec_ID'] == 0x53756e53, 'C_SunSpec_ID != "SunS"'
    for key, value in d.items():
        print('{:16}  {}'.format(key, value))
    print()

    d2 = await c.get_from_mapping(SUNSPEC_INVERTER_MODEL_REGISTER_MAPPINGS)
    for key, value in d2.items():
        print('{:16}  {}'.format(key, value))
    print()


if __name__ == '__main__':
    # Test
    rs = Registers(b'\x41\x61\x70\x00')
    assert rs[0] == 0x4161, rs[0]
    assert rs[1] == 0x7000, rs[1]
    assert rs[:] == (0x4161, 0x7000), rs[:]
    rif = RegisterIface(40000, rs)
    assert rif.get(40000, RType.U32) == 0x41617000, rif.get(40000, RType.U32)
    assert rif.get(40001, RType.U16) == 0x7000, rif.get(40001, RType.U16)
    assert rif.get(40000, RType.STR(4)) == 'Aap', rif.get(40000, RType.STR(4))

    # Output
    if sys.argv[1:2] == ['--publish']:
        sys.stdout.reconfigure(line_buffering=True)  # PYTHONUNBUFFERED
        host, port = sys.argv[2:]  # port defaults to 1502
        asyncio.run(mainloop(host, port))
    else:
        host, port = sys.argv[1:]  # port defaults to 1502
        asyncio.run(oneshot(host, port))
