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
import struct
import sys
from collections import OrderedDict
from decimal import Decimal
from enum import Enum

from pymodbus.client.sync import ModbusTcpClient


class Regs:
    I16 = ('h', 1)  # signed int16
    U16 = ('H', 1)  # unsigned int16
    I32 = ('i', 2)  # signed int32
    U32 = ('I', 2)  # unsigned int32
    BSTR = 'bstr'   # binary string (trailing spaces and NULs)
    STR = 'str'     # string


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
    (40000, 'C_SunSpec_ID', Regs.U32),          # 0x53756e53 "SunS"
    (40002, 'C_SunSpec_DID', Regs.U16),         # 1
    (40003, 'C_SunSpec_Length', Regs.U16),      # 65 + 40003 + 1 == 40069
    (40004, 'C_Manufacturer', Regs.STR, 32),    # "SolarEdge"
    (40020, 'C_Model', Regs.STR, 32),           # "SE5000H-RW000BNN4"
    (40044, 'C_Version', Regs.STR, 16),         # "0004.0011.0030"
    (40052, 'C_SerialNumber', Regs.STR, 16),    # "12345678"
    (40068, 'C_DeviceAddress', Regs.U16),       # 1
    (40069,),                                   # EOF
)

# SolarEdge SE5000H and probably others
# https://www.solaredge.com/sites/default/files/
#   sunspec-implementation-technical-note.pdf
SUNSPEC_INVERTER_MODEL_REGISTER_MAPPINGS = (
    # 40069: [101] = single phase, 102 = split phase, 103 = threephase
    (40069, 'C_SunSpec_DID', Regs.U16),         # 101 (= single phase)
    (40070, 'C_SunSpec_Length', Regs.U16),      # 50 + 40070 + 1 == 40121
    (40083, 'I_AC_Power', Regs.U16),            # AC Power value [Watt]
    (40084, 'I_AC_Power_SF', Regs.I16),         # AC Power scale (exp)
    (40085, 'I_AC_Frequency', Regs.U16),        # AC Frequency value [Hertz]
    (40086, 'I_AC_Frequency_SF', Regs.I16),     # AC Frequency scale (exp)
    (40093, 'I_AC_Energy_WH', Regs.U32),        # AC Lifetime Energy prod [WH]
    (40095, 'I_AC_Energy_WH_SF', Regs.I16),     # AC Lifetime Energe sc. (exp)
    (40100, 'I_DC_Power', Regs.U16),            # DC Power value [Watt]
    (40101, 'I_DC_Power_SF', Regs.I16),         # DC Power scale (exp)
    (40103, 'I_Temp_Sink', Regs.I16),           # Heat Sink temp [C]
    (40106, 'I_Temp_Sink_SF', Regs.I16),        # Heat Sink scale (exp)
    (40107, 'I_Status', SunspecInverterStatus),  # Operating State
    (40108, 'I_Status_Vendor', Regs.U16),       # Vendor-defined state
    (40121,),                                   # EOF
)


class RegsImpl(Regs):
    """
    Implementation converting registers (list of 16bit integers) with an
    offset to other types.

    Allowed types are defined in the Regs superclass.

    Example usage::

        r = RegsImpl(40000, [0x1234, 0x5678])
        r.get(40000, Regs.U32) == 0x12345678
    """
    @staticmethod
    def ushorts2bin(registers, off, size):
        return struct.pack(
            '>{}'.format('H' * size), *registers[off:off + size])

    @classmethod
    def ushorts2type(cls, type_, registers, off, size):
        return struct.unpack(
            '>{}'.format(type_), cls.ushorts2bin(registers, off, size))[0]

    def __init__(self, offset, u16values):
        """
        Create RegsImpl storing the offset so get() can use absolute offsets
        """
        self._off = offset
        self._u16values = u16values

    def get(self, offset, type_, *args):
        """
        Converts the registers to the specified type

        type_ must be a predefined type known to this class or a
        subclass of Enum. Some types, like strings, take an additional
        (length) argument.
        """
        if isinstance(type_, type) and issubclass(type_, Enum):
            uint16 = self.ushorts2type(
                'H', self._u16values, offset - self._off, 1)
            return type_(uint16)
        elif isinstance(type_, tuple):
            assert not args
            inttype, intsize = type_
            return self.ushorts2type(
                inttype, self._u16values, offset - self._off, intsize)
        elif type_ == 'bstr':
            assert len(args) == 1, args
            length = args[0]
            assert length % 2 == 0, length
            return self.ushorts2bin(
                self._u16values, offset - self._off, length // 2)
        elif type_ == 'str':
            assert len(args) == 1, args
            length = args[0]
            return (
                self.get(offset, 'bstr', length)
                .rstrip(b'\x00\x20').decode('utf-8'))
        else:
            raise NotImplementedError((offset, type_, args))

    def mapping2dict(self, mapping):
        """
        Take a list of (offset, name, type) tuples, return an OrderedDict

        Example usage::

            MAPPING = [
                (40000, 'C_SunSpec_ID', Regs.U32),
            ]
            r.mapping2dict(MAPPING)
            # ^- returns a dict with key 'C_SunSpec_ID' and the unsigned
            #    int32 taken from offset 40000.
        """
        ret = OrderedDict()
        assert mapping and len(mapping[-1]) == 1, 'expected END marker'
        for item in mapping[0:-1]:  # don't fetch END marker
            offset, name, type_and_args = item[0], item[1], item[2:]
            ret[name] = self.get(offset, *type_and_args)
        return ret


class SunspecRegs(RegsImpl):
    def mapping2dict(self, mapping):
        """
        Extend RegsImpl to automatically handle "_SF" (scale factor) keys

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
            ret[key] = Decimal(ret[key]) * exp
            del ret[sf_key]
        return ret


class SunspecModbusTcpClient(ModbusTcpClient):
    def get_from_mapping(self, mapping, unit=1):
        first_reg = mapping[0][0]
        eof_reg = mapping[-1][0]
        count = eof_reg - first_reg
        regs = self.read_holding_registers(first_reg, count=count, unit=unit)
        sunspecregs = SunspecRegs(first_reg, regs.registers)
        return sunspecregs.mapping2dict(mapping)


if __name__ == '__main__':
    # Test
    r = RegsImpl(40000, [0x1234, 0x5678])
    assert r.get(40000, Regs.U32) == 0x12345678

    # Output
    host, port = sys.argv[1:]  # port defaults to 1502
    c = SunspecModbusTcpClient(host=host, port=port, timeout=15)

    d = c.get_from_mapping(SUNSPEC_COMMON_MODEL_REGISTER_MAPPINGS)
    assert d['C_SunSpec_ID'] == 0x53756e53, 'C_SunSpec_ID != "SunS"'
    for key, value in d.items():
        print('{:16}  {}'.format(key, value))
    print()

    d2 = c.get_from_mapping(SUNSPEC_INVERTER_MODEL_REGISTER_MAPPINGS)
    for key, value in d2.items():
        print('{:16}  {}'.format(key, value))
    print()
