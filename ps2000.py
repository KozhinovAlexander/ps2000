#!/usr/bin/env python3

'''
Python Interface for EA-PS2000 series power supply units.
'''

import argparse
import serial
import struct
import time


class ps2000(object):
    PS_QUERY = 0x40
    PS_SEND = 0xc0

    def __init__(self, port:str, timeout:float=0.06, baudrate:int=115200,
                 parity:str=serial.PARITY_ODD, verbosity_level=0):
        '''
        Initialize the PS2000 device with the specified serial port settings.
        Args:
            port (str): The serial port to which the PS2000 device is connected.
            timeout (float, optional): The timeout for serial communication in seconds. Default is 0.06.
            baudrate (int, optional): The baud rate for serial communication. Default is 115200.
            parity (str, optional): The parity bit setting for serial communication. Default is serial.PARITY_ODD.
            verbosity_level (int, optional): The verbosity level for logging. Use 3 to see more information. Default is 0.
        Attributes:
            _verbose (int): Stores the verbosity level for logging.
            ser_dev (serial.Serial): The serial device object for communication with the PS2000.
            _u_nom (float): The nominal voltage of the PS2000 device.
            _i_nom (float): The nominal current of the PS2000 device.
        '''
        self._verbosity_lvl = verbosity_level
        # set timeout to 0.06s to guarantee minimum interval time of 50ms
        self.ser_dev = serial.Serial(port, timeout=timeout, baudrate=baudrate,
                                     parity=parity)
        self._u_nom = self.get_nominal_voltage()
        self._i_nom = self.get_nominal_current()

    def __enter__(self):
        self.set_remote(True)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.set_remote(False)
        self.ser_dev.close()
        if exc_type is not None:
            print(f'An exception occurred: {exc_value}\nTraceback: {traceback}')

    @staticmethod
    def _construct_telegram(telegram_type, node, obj, data) -> bytearray:
        '''
        Constructs a telegram message for communication.
        Args:
            telegram_type (int): The type of the telegram.
            node (int): The device node.
            obj (int): The object identifier.
            data (bytes): The data to be included in the telegram.
        Returns:
            bytearray: The constructed telegram message.
        The telegram message format is as follows:
            - Start delimiter (SD): 0x30 + telegram_type
            - Device node (DN): node
            - Object (OBJ): obj
            - Data (DATA): data (if any)
            - Checksum (CS): sum of all previous bytes, split into two bytes (CS0 and CS1)
        '''
        telegram = bytearray()
        telegram.append(0x30 + telegram_type) # SD (start delimiter)
        telegram.append(node)                 # DN (device node)
        telegram.append(obj)                  # OBJ (object)
        if len(data) > 0:                     # DATA
            telegram.extend(data)
            telegram[0] += len(data) - 1  # update length

        cs = sum(telegram)
        telegram.append(cs >> 8)   # CS0
        telegram.append(cs & 0xff) # CS1 (checksum)

        return telegram

    @staticmethod
    def _checksum_verify(ans):
        '''
        Compare checksum with header and data in response from device.
        This function calculates the checksum of the given response data and
        compares it with the checksum provided in the last two bytes of the response.
        If the calculated checksum does not match the provided checksum, an assertion
        error is raised indicating a checksum mismatch.
        Args:
            ans (list or bytearray): The response data from the device, including the
                                     checksum in the last two bytes.
        Raises:
            AssertionError: If the calculated checksum does not match the provided checksum.
        '''
        cs = sum(ans[0:-2])
        assert ans[-2] == (cs >> 8) and ans[-1] == (cs & 0xff), \
            'Checksum mismatch'

    @staticmethod
    def _check_error(ans):
        '''
        Checks for errors in the given answer and raises an assertion error with
        a detailed message if an error is found.
        Args:
            ans (list of int): The answer to check, represented as a list of integers.
        Raises:
            AssertionError: If an error is found, with a message detailing the
                            error type and the answer in hexadecimal format.
        '''
        if ans[2] != 0xff:
            return

        response_state_table = {
            0x00: 'OK: Acknowledge',  # got acknowledge - not an error
            0x03: 'ERROR: Com: Checksum incorrect',
            0x04: 'ERROR: Com: Start delimiter incorrect',
            0x05: 'ERROR: Com: Wrong address for output',
            0x07: 'ERROR: Com: Object not defined',
            0x08: 'ERROR: Usr: Object length incorrect',
            0x09: 'ERROR: Usr: Access denied',
            0x0f: 'ERROR: Usr: Device is locked',
            0x30: 'ERROR: Usr: Upper limit exceeded',
            0x31: 'ERROR: Usr: Lower limit exceeded',
        }

        resp_str = 'unknown error'
        if ans[3] in response_state_table.keys():
            resp_str = response_state_table[ans[3]]
        assert ans[3] == 0x00, f'{resp_str}\n--answer:\t\t{ps2000.bytes2hex(ans)}'

    @staticmethod
    def bytes2hex(bytes_arr):
        '''
        Converts a byte array to a string of hexadecimal values.
        Args:
            bytes_arr (bytes): The byte array to convert.
        Returns:
            str: A string of hexadecimal values separated by spaces.
        '''
        return ' '.join(hex(b) for b in bytes_arr)

    def _transfer(self, telegram_type, node, obj, data,
                  read_buff_len:int=100) -> bytes:
        '''
        Transfers data to and from a serial device.
        This method constructs a telegram based on the provided telegram_type, node, object, and data,
        sends it to the serial device, and reads the response. It also performs verbosity-based
        logging, checks the response length, and validates the checksum and error status.
        Args:
            telegram_type (int): The type of telegram to construct.
            node (int): The node identifier.
            obj (int): The object identifier.
            data (bytes): The data to be included in the telegram.
            read_buff_len(int): Bytes read buffer length.
        Returns:
            bytes: The response received from the serial device.
        Raises:
            SystemExit: If the response received is shorter than expected.
        '''
        telegram = ps2000._construct_telegram(telegram_type, node, obj, data)

        if self._verbosity_lvl >= 3:
            print(f'-- telegram:\t\t{ps2000.bytes2hex(telegram)}')

        self.ser_dev.write(telegram)  # send telegram
        ans = self.ser_dev.read(read_buff_len)

        if self._verbosity_lvl >= 3:
            print(f'-- answer:\t\t{ps2000.bytes2hex(telegram)}')

        min_len = 5  # 5 bytes is the minimum length of a valid answer
        assert len(ans) >= min_len, \
            f'Short answer {len(ans)} bytes received, expected at least {min_len} bytes)'

        # check the answer
        ps2000._checksum_verify(ans)
        ps2000._check_error(ans)

        return ans

    def _read_obj(self, obj, obj_type: type = bytes):
        allowed_obj_types = [bytes, str, float, int]
        assert obj_type in allowed_obj_types, \
            f'ERROR: Object type shall be one of {allowed_obj_types} ' \
            f'but it is {obj_type if obj_type is not type else type(obj_type)}'

        msg = self._transfer(self.PS_QUERY, 0, obj, '')[3:-2]
        if obj_type is bytes:
            return msg
        elif obj_type is str:
            return msg.decode('ascii')
        elif obj_type is float:
            return struct.unpack('>f', msg)[0]
        elif obj_type is int:
            return struct.unpack('>H', msg)[0]
        else:
            assert False, 'ERROR: Unknown error!'

    def _write_obj(self, obj, data, obj_type: type = bytes, mask = None):
        allowed_obj_types = [bytes, int]
        assert obj_type in allowed_obj_types, \
            f'ERROR: Object type shall be one of {allowed_obj_types} ' \
            f'but it is {obj_type if obj_type is not type else type(obj_type)}'

        if obj_type is bytes:
            assert mask is not None, f'ERROR: The mask argument value {mask} is not allowed!'
            ans = self._transfer(self.PS_SEND, 0, obj, [mask, data])
            return ans[3:-2]
        elif obj_type is int:
            ans = self._transfer(self.PS_SEND, 0, obj, [data >> 8, data & 0xff])
            return (ans[3] << 8) + ans[4]
        else:
            assert False, 'ERROR: Unknown error!'

    # object 0
    def get_type(self):
        return self._read_obj(0, str)

    # object 1
    def get_serial(self):
        return self._read_obj(1, str)

    # object 2
    def get_nominal_voltage(self):
        return float(self._read_obj(2, float))

    # object 3
    def get_nominal_current(self):
        return float(self._read_obj(3, float))

    # object 4
    def get_nominal_power(self):
        return self._read_obj(4, float)

    # object 6
    def get_article(self):
        return self._read_obj(6, str)

    # object 8
    def get_manufacturer(self):
        return self._read_obj(8, str)

    # object 9
    def get_version(self):
        return self._read_obj(9, str)

    # object 19
    def get_device_class(self):
        return self._read_obj(19, int)

    # object 38
    def get_OVP_threshold(self):
        v = self._read_obj(38, int)
        return ps2000.percent2real(self._u_nom, float(v), 25600.0)

    def set_OVP_threshold(self, u):
        return self._write_obj(38, u, int)

    # object 39
    def get_OCP_threshold(self):
        i = self._read_obj(39, int)
        return ps2000.percent2real(self._i_nom, float(i), 25600.0)

    def set_OCP_threshold(self, i):
        return self._write_obj(39, i, int)

    # object 50
    def get_voltage_setpoint(self):
        v = self._read_obj(50, int)
        return ps2000.percent2real(self._u_nom, float(v), 25600.0)

    def set_voltage(self, u):
        return self._write_obj(50, int(round((u * 25600.0) / self._u_nom)), int)

    # object 51
    def get_current_setpoint(self):
        i = self._read_obj(51, int)
        return ps2000.percent2real(self._i_nom, float(i), 25600.0)

    def set_current(self, i):
        return self._write_obj(51, int(round((i * 25600.0) / self._i_nom)), int)

    # object 54
    def get_control(self):
        ans = bytes(self._read_obj(54))
        control = {
            'output_on': True if ans[1] & 0x01 else False,
            'remote': True if ans[0] & 0x01 else False
        }
        return control

    def _set_control(self, mask, data):
        ans = bytes(self._write_obj(54, data, bytes, mask))
        # return True if command was acknowledged ("error 0")
        return ans[0] == 0xff and ans[1] == 0x00

    def get_remote(self):
        return self.get_control()['remote']

    def set_remote(self, remote=True):
        if remote:
            return self._set_control(0x10, 0x10)
        else:
            return self._set_control(0x10, 0x00)

    def set_local(self, local=True):
        return self.set_remote(not local)

    def get_output_on(self):
        return self.get_control()['output_on']

    def set_output_on(self, on=True):
        if on:
            return self._set_control(0x01, 0x01)
        else:
            return self._set_control(0x01, 0x00)

    def set_output_off(self, off=True):
        return self.set_output_on(not off)

    @staticmethod
    def percent2real(rated_value:float, percent_actual_value:float,
                     translation_factor:float) -> float:
        '''
        Convert percent value to real one by utilizing following equation:
            real_actual_value = (rated_value * percent_actual_value) / translation_factor
        '''
        return (rated_value * percent_actual_value) / translation_factor

    # object 71
    def get_actual(self):
        ans = bytes(self._read_obj(71))
        state = {
            'remote': True if ans[0] & 0x03 else False,
            'on': True if ans[1] & 0x01 else False,
            'CC': True if ans[1] & 0x06 else False,
            'CV': False if ans[1] & 0x06 else True, # not CC
            'tracking': True if ans[1] & 0x08 else False,
            'OVP': True if ans[1] & 0x10 else False,
            'OCP': True if ans[1] & 0x20 else False,
            'OPP': True if ans[1] & 0x40 else False,
            'OTP': True if ans[1] & 0x80 else False,
            'v': ps2000.percent2real(self._u_nom, float((int(ans[2]) << 8) + int(ans[3])),25600.0),
            'i': ps2000.percent2real(self._i_nom, float((int(ans[4]) << 8) + int(ans[5])),25600.0),
        }
        return state

    # object 72
    def get_setpoints(self):
        ans = bytes(self._read_obj(72))
        state = {
            'remote': True if ans[0] & 0x03 else False,
            'on': True if ans[1] & 0x01 else False,
            'CC': True if ans[1] & 0x06 else False,
            'OVP': True if ans[1] & 0x10 else False,
            'OCP': True if ans[1] & 0x20 else False,
            'OPP': True if ans[1] & 0x40 else False,
            'OTP': True if ans[1] & 0x80 else False,
            'v': ps2000.percent2real(self._u_nom, float((int(ans[2]) << 8) + int(ans[3])),25600.0),
            'i': ps2000.percent2real(self._i_nom, float((int(ans[4]) << 8) + int(ans[5])),25600.0),
        }
        return state


@staticmethod
def print_info(ps):
    print(
        f'type    {ps.get_type()}\n' \
        f'serial  {ps.get_serial()}\n' \
        f'article {ps.get_article()}\n' \
        f'manuf   {ps.get_manufacturer()}\n' \
        f'version {ps.get_version()}\n' \
        f'nom. voltage {ps.get_nominal_voltage()}\n' \
        f'nom. current {ps.get_nominal_current()}\n' \
        f'nom. power   {ps.get_nominal_power()}\n' \
        f'class        {hex(ps.get_device_class())}\n' \
        f'OVP          {ps.get_OVP_threshold()}\n' \
        f'OCP          {ps.get_OCP_threshold()}\n' \
        f'control      {hex(ps.set_remote())}' \
    )
    state = ps.get_actual()
    print(f'-- state: {state}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Control PS2000 power supply')
    parser.add_argument(
        '-p', '--port', type=str, help='serial port to use', required=True)

    default_voltage = None
    default_current = None
    parser.add_argument('-V', '--voltage', type=float, default=default_voltage,
                        required=False,
                        help=f'Voltage to be set. Nothing will be changed if None.' \
                             f'(default: {default_voltage})')
    parser.add_argument('-I', '--current', type=float, default=default_current,
                        required=False,
                        help=f'Current to be set. Nothing will be changed if None.' \
                             f'(default: {default_current})')

    group_verb = parser.add_mutually_exclusive_group(required=False)
    group_verb.add_argument('-v', dest='verbose', action='count', default=0,
                            help='increase verbosity level')

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--on', help='turn on', action='store_true')
    group.add_argument('--off', help='turn off', action='store_true')
    group.add_argument('--toggle', help='toggle', action='store_true')
    group.add_argument('--info', help='toggle', action='store_true')
    args = parser.parse_args()

    with ps2000(args.port, verbosity_level=args.verbose) as ps:
        print(f"Vset: {ps.get_voltage_setpoint()}")
        print(f"Iset: {ps.get_current_setpoint()}")
        print(f"Vact: {ps.get_actual()['v']}")
        print(f"Iact: {ps.get_actual()['i']}")

        if isinstance(args.voltage, float):
            print(f'Set voltage: {args.voltage}')
            ps.set_voltage(args.voltage)

        if isinstance(args.current, float):
            print(f'Set current: {args.current}')
            ps.set_current(args.current)

        if args.on:
            print('Turning on')
            ps.set_output_on(True)
        elif args.off:
            print('Turning off')
            ps.set_output_on(False)
        elif args.toggle:
            output_on = ps.get_output_on()
            print(f"Output {'on' if output_on else 'off'} -> " \
                  f"turning {'on' if not output_on else 'off'}")
            ps.set_output_on(not output_on)
        elif args.info:
            print_info(ps)

        time.sleep(0.5)  # wait for settling the values
        print(f"Vset: {ps.get_voltage_setpoint()}")
        print(f"Iset: {ps.get_current_setpoint()}")
        print(f"Vact: {ps.get_actual()['v']}")
        print(f"Iact: {ps.get_actual()['i']}")
