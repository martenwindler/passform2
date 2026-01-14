import unittest
import time
from rpi_i2c.i2c_types import AnalogIn

test_values = 5*[0] # input as currently attached to board
#test_values[0] = 0.5  # example: input I4 is at 50%
#test_values[4] = 0  # example: input I4 is at 50%


class TestAnalogIn(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._address = 0x08 # CHANGE - DONE is 08
        cls._length = 5 # could be set as default in class
        cls._sleeptime = 0.1    # wait between individual tests
        cls._client = AnalogIn(cls._address, cls._length)

    # def test_input_full(self):
    #     """ Test each analog input at 100% as subtest (#: 1..n) """
    #     for i in range(1, self._length+1):
    #         with self.subTest(i=i):
    #             in_val = self._client.read(i)
    #             self.assertEqual(in_val, 1)
    #             time.sleep(self._sleeptime)

    # def test_input_zero(self):
    #     """ Test each analog input at 0% as subtest (#: 1..n) """
    #     for i in range(1, self._length+1):
    #         with self.subTest(i=i):
    #             in_val = self._client.read(i)
    #             self.assertEqual(in_val, 0)
    #             time.sleep(self._sleeptime)

    def test_input_as_specified(self):
        """ Test each input at ON as subtest (#: 1..n) """
        for idx, val in enumerate(test_values):
            with self.subTest(i=idx):
                in_val = self._client.read(idx+1)
                self.assertAlmostEqual(in_val, val, 1) #  has to be similar when rounded to 1 place
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
