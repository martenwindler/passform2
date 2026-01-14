import unittest
import time
from rpi_i2c.i2c_types import AnalogOut

class TestAnalogOut(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._address = 0x58 # CHANGE - DONE is 58
        cls._length = 4 # could be set as default in class
        cls._sleeptime = 0.1    # wait between individual tests
        cls._client = AnalogOut(cls._address, cls._length)
        # assert that all outputs are OFF

    # @classmethod
    # def tearDownClass(cls):
    #     cls._client.destroy()

    def test_array_full(self):
        """ Write 1 on all outputs """
        self._client.write(self._length*[1])
        time.sleep(self._sleeptime)
        # all outputs should be 100%

    def test_array_zero(self):
        """ Write 0 on all outputs """
        self._client.write(self._length*[0])
        time.sleep(self._sleeptime)
        # all outputs should be 0%

    def test_single_full(self):
        """ Test each analog input at 100% as subtest (#: 1..n) """
        for i in range(1, self._length+1):
            with self.subTest(i=i):
                self._client.write(pin=i, val=1)
                time.sleep(self._sleeptime)
                # pin i should equal 1

    def test_single_zero(self):
        """ Test each analog input at 0% as subtest (#: 1..n) """
        for i in range(1, self._length+1):
            with self.subTest(i=i):
                self._client.write(pin=i, val=0)
                time.sleep(self._sleeptime)
                # pin i should equal 0

    def test_array_faulty_size(self):
        """ Test ill-formed arrays: size """
        for idx, len in enumerate([-1,0,self._length+1,self._length+10]):
            with self.subTest(i=idx):
                with self.assertRaises(AssertionError):
                    self._client.write(len*[False])

    def test_faulty_int_val(self):
        """ Test ill-formed arrays: wrong int value """
        wrong_values = [-3,-1]
        for idx, val in enumerate(wrong_values):
            with self.subTest(i=idx):
                with self.assertRaises(AssertionError):
                   self._client.write(pin=1, val=val)

    def test_faulty_type(self):
        """ Test ill-formed arrays: wrong type """
        wrong_values = [-.1, {'key': 'value'}, {1}, {1,2,3}, None]
        for idx, val in enumerate(wrong_values):
            with self.subTest(i=idx):
                with self.assertRaises(AssertionError):
                   self._client.write(pin=1, val=val)

if __name__ == '__main__':
    unittest.main()
