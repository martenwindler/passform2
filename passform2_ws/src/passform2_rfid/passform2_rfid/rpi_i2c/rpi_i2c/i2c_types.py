import smbus2
import time
from types import *
import bitarray
import bitarray.util

class DigitalIn(object):
    """docstring for DigitalIn."""

    def __init__(self, address:int = 0x38, length:int = 8, i2c_portNumber:int = 1):
        super(DigitalIn, self).__init__()
        self._bus = smbus2.SMBus(i2c_portNumber) #smbus number (standard=1 in new models)
        self._address = address #i2c adress of DI-board
        self._length = length #number of INs

    def read_all(self) -> list:
        """Reads all bit-inputs as single integer and returns the individual bit-values as array"""
        in_byte = self._bus.read_byte_data(self._address,255) # reads 8 INs as Integer(from binary with E8 as most significant bit)
        ba = ~bitarray.util.int2ba(in_byte, length=self._length, endian='little') # int to binary and inverted
        return list(ba)

    def read(self, pin:int) -> int:
        """Reads all bit-inputs and returns the specified pin value"""
        assert pin in range(1, self._length+1), f'pin must be in 1 .. {self._length}, got {pin}'
        return self.read_all()[pin-1] # single pin value

class AnalogIn(object):
    """docstring for AnalogIn. https://www.horter.de/blog/i2c-analog-input-modul-mit-python-und-tkinter-am-raspberry-pi/"""

    def __init__(self, address:int = 0x08, length:int = 5, i2c_portNumber:int = 1):
        super(AnalogIn, self).__init__()
        self._bus = smbus2.SMBus(i2c_portNumber)
        self._address = address #i2c adress of AI card
        self._length = length # pin length to read (block is twice the size)
        self._offset = 0 # block_read offset (should always be zero. here to prevent magic number)

    def read_all(self) -> list[float]:
        '''Read all analog in pin and return their voltage level in percent 0..1'''
        # read inputs as array (block length is twice the pin length)
        byte_array = self._bus.read_i2c_block_data(self._address, self._offset, self._length*2)
        return [self.bit_to_val(byte_array[i], byte_array[i+1]) for i in range(0, len(byte_array), 2)]

    def read(self, pin:int) -> float:
        '''Read a single analog_in pin and return its voltage level in percent 0..1'''
        # TODO: perform smaller read with offset and length=2
        assert pin in range(1, self._length+1), f'pin must be in 1 .. {self._length}, got {pin}'
        return self.read_all()[pin-1]

    @staticmethod
    def bit_to_val(lb:int, hb:int) -> float:
        ''' convert lowbyte, highbyte to a float'''
        val = lb | hb << 8 # combine low and highbit
        # val = lb + hb*256
        return val/1023.0 # calculate percentage (0 to 1) - maybe 1000 is the limit and scale

class DigitalOut(object):
    """docstring for DigitalOut. https://www.horter.de/blog/i2c-module-mit-python-und-tkinter-auf-raspberry-py/"""

    def __init__(self, address:int = 0x20, length:int = 8, i2c_portNumber:int = 1):
        super(DigitalOut, self).__init__()
        self._bus = smbus2.SMBus(i2c_portNumber)
        self._address = address
        self._length = length
        self.clear_values()

    def clear_values(self) -> None:
        """ clear all outputs to off and create internal value storage """
        self.valuesArray = bitarray.bitarray(self._length*[0], endian='little') #set bitarray of zeros
        self._write_out() # write data

    def write(self, val: int|list[int], pin:int=None) -> None:
        self.assert_val(val, pin)
        if pin is None:
            """ set bitmap as val assuming val is list """
            self.valuesArray = bitarray.bitarray(val, endian='little') #create bitarray from val
        else:
            """ set bitmap at pin to val """
            self.valuesArray[pin-1] = val #set single value of bitarray
        # write data
        self._write_out()

    def _write_out(self) -> None:
        self.bit_out_values = bitarray.util.ba2int(~self.valuesArray) #convert int for internal logic
        self._bus.write_byte(self._address, self.bit_out_values) #write to OUTs

    def assert_val(self,val: int|list[int] ,pin:int) ->None:
        if pin is None:
            # write array
            assert type(val) is list, f'singlevariable must be list, got {type(val)}'
            assert len(val) == self._length, f'arraylength must be {self._length}, got {len(val)}'
        else:
            # write individual pin
            val = [val]
            assert pin in range(1,self._length+1), f'bit position must be in 1 .. {self._length}, got {pin}'
        for v in val:
            assert v in [0,1], f'bit value must be 0 or 1, got {val}'

class AnalogOut(object):
    """docstring for AnalogOut. https://www.horter.de/blog/i2c-analog-output-modul-mit-python-und-tkinter-am-raspberry-pi/"""

    def __init__(self, address:int = 0x58, length:int = 4, i2c_portNumber:int = 1):
        super(AnalogOut, self).__init__()
        self._bus = smbus2.SMBus(i2c_portNumber)
        self._address = address
        self._length = length
        self.clear_values()

    def clear_values(self) -> None:
        self.write(self._length*[1.0]) # set array of ones (LEDs max brigthness at 0)

    def write(self, val:float|list[float], pin:int=None) -> None:
        self.assert_val(val, pin)   #check for rigth format
        if pin is None:
            """ write array """
            self.valuesArray = val
            for idx, x in enumerate(self.valuesArray):
                self._bus.write_i2c_block_data(self._address, idx, self.val_to_bit(x))
        else:
            """ pin write """
            self.valuesArray[pin-1] = val #set single value of bitarray
            self._bus.write_i2c_block_data(self._address, pin-1, self.val_to_bit(val))
        time.sleep(0.02) #sleep for internal i2cboard voltage-changes

    def assert_val(self, val:float|list[float], pin:int) -> None:
        if pin is None:
            # write array
            assert type(val) is list, f'singlevariable must be list, got {type(val)}'
            assert len(val) == self._length, f'arraylength must be {self._length}, got {len(val)}'
        else:
            # write individual pin
            val = [val]
            assert pin in range(1,self._length+1), f'bit position must be in 1 .. {self._length}, got {pin}'
        for v in val:
            self.assert_value(v)

    @staticmethod
    def assert_value(v):
        '''Asserts the value is a float in 0..1'''
        assert type(v) == float, f'value must be float, got {type(v)}'
        assert 0.0<=v<=1.0, f'bit value must be between 0 and 1, got {v}'

    def val_to_bit(self, x:float) ->list[int]:
        '''
        Convert a float value [0..1] to 10 bit array as [LowByte,HighByte].
        Performs value assertion.
        '''
        self.assert_value(x)
        a = x*1023.0
        HBy = int(a/256.0)
        LBy = int(a-HBy*256.0)
        field = [LBy, HBy]
        return field

if __name__ == '__main__':

    main()
    ai=AnalogIn()
    ai.read_all()
    # ai.read(5)
    # q=0.00
    # ao = AnalogOut()
    # print(ao.valuesArray)
    # time.sleep(q)
    # ao.write([1,1,1,0])
    # print(ao.valuesArray)
    # time.sleep(q)
    # ao.write(0,3)
    # ao.write(0.105,4)
    # print(ao.valuesArray)
    # time.sleep(q)
    # ao.write(0.12,4)
    # print(ao.valuesArray)
    # time.sleep(q)
    # ao.write(1,4)
    # print(ao.valuesArray)
    # time.sleep(q)
    # ao.write([0,1,0,1])
    # print(ao.valuesArray)
    # time.sleep(q)
    # ao.write([1,0.12,1,0.12])
    # print(ao.valuesArray)
    # time.sleep(q)
    # ao.write([0,0,0,0])
    # print(ao.valuesArray)
