from rpi_i2c_msgs.msg import AnalogArray, DigitalArray

def array_to_dict(arr:AnalogArray|DigitalArray) -> dict:
    return {i.pin:i.data for i in arr.data}
