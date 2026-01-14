import unittest
import time
from rpi_i2c.i2c_types import DigitalIn

test_values = 8*[0] # input as currently attached to board
#test_values[4] = 1  # example: input I5 is ON

class TestDigitalIn(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._address = 0x38 # CHANGE - DONE
        cls._length = 8 # could be set as default in class
        cls._sleeptime = 0.1    # wait between individual tests
        cls._client = DigitalIn(cls._address, cls._length)

    # def test_input_full(self):
    #     """ Test each input at ON as subtest (#: 1..n) """
    #     for i in range(1, self._length+1):
    #         with self.subTest(i=i):
    #             in_val = self._client.read(i)
    #             self.assertEqual(in_val, 1)
    #             time.sleep(self._sleeptime)
    #
    # def test_input_zero(self):
    #     """ Test each input at OFF as subtest (#: 1..n) """
    #     for i in range(1, self._length+1):
    #         with self.subTest(i=i):
    #             in_val = self._client.read(i)
    #             self.assertEqual(in_val, 0)
    #             time.sleep(self._sleeptime)

    def test_read_all(self):
        """ Test all inputs as array """
        self.assertEqual(test_values, self._client.read_all())

    def test_input_as_specified(self):
        """ Test each input at ON as subtest (#: 1..n) """
        for idx, val in enumerate(test_values):
            with self.subTest(i=idx):
                in_val = self._client.read(idx+1)
                self.assertEqual(in_val, val)
                time.sleep(self._sleeptime)

    def test_faulty_pin(self):
        """ Test bad pins """
        wrong_pins = [-1, self._length+1, None, 'three']
        for idx, pin in enumerate(wrong_pins):
            with self.subTest(i=idx):
                with self.assertRaises(AssertionError):
                   self._client.read(pin)

if __name__ == '__main__':
    unittest.main()
