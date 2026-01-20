import unittest
import uuid

import std_msgs.msg
import passform_msgs.msg
from passform_util.types.location import Location

class TestLocation(unittest.TestCase):
    """Test Item from inventory.py
    """

    def test_init(self):
        """Test constructor"""
        msg = passform_msgs.msg.Location(
            header = std_msgs.msg.Header(
                frame_id = str(uuid.uuid4())
            ),
            aoi = passform_msgs.msg.AreaOfInterest(
                label = str(uuid.uuid4())
            )
        )
        loc = Location(msg)
        self.assertEqual(loc.get_frame_id(), msg.header.frame_id)
        self.assertEqual(loc.get_aoi().get_label(), msg.aoi.label)

if __name__ == '__main__':
    unittest.main()