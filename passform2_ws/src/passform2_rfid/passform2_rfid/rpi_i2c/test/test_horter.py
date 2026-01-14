import unittest
import time
from rpi_i2c.i2c_types import AnalogIn

class TestAnalogin(unittest.TestCase):

    self._address = 0x30 # CHANGE
    self._length = 8 # could be set as default in class
    self._sleeptime = 0.1    # wait between individual tests

    @classmethod
    def setUpClass(cls):
        cls._client = AnalogIn(self._address, self._length)

    # @classmethod
    # def tearDownClass(cls):
    #     cls._client.destroy()

    def test_input_full(self):
        """ Test each analog input at 100% as subtest (#: 1..n) """
        for i in range(1, self._length+1):
            with self.subTest(i=i):
                val = self._client.read(i)
                self.assertEqual(val, 1)
                time.sleep(self._sleeptime)

    def test_input_zero(self):
        """ Test each analog input at 0% as subtest (#: 1..n) """
        for i in range(1, self._length+1):
            with self.subTest(i=i):
                val = self._client.read(i)
                self.assertEqual(val, 0)
                time.sleep(self._sleeptime)

class DigitalOut(unittest.TestCase):

    self._address = 0x30 # CHANGE
    self._length = 8 # could be set as default in class
    self._sleeptime = 0.1    # wait between individual tests

    @classmethod
    def setUpClass(cls):
        cls._client = DigitalOut(self._address, self._length)
        # assert that all outputs are OFF

    # @classmethod
    # def tearDownClass(cls):
    #     cls._client.destroy()

    def test_array_full(self):
        """ Write 1 on all outputs """
        self._client.write(self._length*[1])
        time.sleep(self._sleeptime)
        # all outputs should be ON

    def test_array_zero(self):
        """ Write 1 on all outputs """
        self._client.write(self._length*[0])
        time.sleep(self._sleeptime)
        # all outputs should be OFF

    def test_single_full(self):
        """ Test each digital output at 1 as subtest (#: 1..n) """
        for i in range(1, self._length+1):
            with self.subTest(i=i):
                self._client.write(pin=i, val=1)
                time.sleep(self._sleeptime)
                # pin i should be ON

    def test_single_zero(self):
        """ Test each digital output at 0 subtest (#: 1..n) """
        for i in range(1, self._length+1):
            with self.subTest(i=i):
                self._client.write(pin=i, val=0)
                time.sleep(self._sleeptime)
                # pin i should be OFF

    def test_single_bool(self):
        """ Test each digital output with a toggled bool """
        for i in range(1, self._length+1):
            with self.subTest(i=i):
                self._client.write(pin=i, val=True)
                time.sleep(self._sleeptime)
                self._client.write(pin=i, val=False)
                time.sleep(self._sleeptime)
                # pin i should go ON -> OFF

    def test_array_bool(self):
        """ Test each digital output with a toggled bool """
        self._client.write(self._length*[True])
        time.sleep(self._sleeptime)
        self._client.write(self._length*[False])
        time.sleep(self._sleeptime)
        # all outputs should go ON -> OFF
        
if __name__ == '__main__':
    unittest.main()
