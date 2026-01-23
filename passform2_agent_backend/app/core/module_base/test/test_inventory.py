import unittest
import uuid

import passform_msgs.msg
from module_base.inventory import ModuleInventory, Storage
from passform_util.types import Item, Location


class TestItem(unittest.TestCase):
    """Test Item from inventory.py
    """

    def test_init(self):
        """Test constructor"""
        name = str(uuid.uuid4()) # random string
        uid = str(uuid.uuid4()) # random string
        item = Item(name=name, uid=uid)
        self.assertEqual(item.get_name(), name)
        self.assertEqual(item.get_uid(), uid)

    def test_init_negative_quantity(self):
        """Quantity must never be negative"""
        with self.assertRaises(AssertionError):
            Item(name='test', uid='1', quantity=-1)

    def test_conversion(self):
        """Test ROS message conversion"""
        item = Item(name='item', uid='1', quantity=123)
        item_msg = item.to_msg()
        self.assertIsInstance(item_msg, passform_msgs.msg.Item)
        item_converted = Item.from_msg(item_msg)
        self.assertEqual(item, item_converted)

    def test_add(self):
        """Add item quantity to new item"""
        quantity = 1
        item1 = Item(name='item', uid='1', quantity=quantity)
        item2 = Item(name='item', uid='1', quantity=quantity)
        item1.add(item2)
        self.assertEqual(item1.get_quantity(), quantity+quantity)
        self.assertEqual(item2.get_quantity(), quantity-quantity)

    def test_faulty_add(self):
        """Add item to different item"""
        quantity=1
        item1 = Item(name='item', uid='1', quantity=quantity)
        item2 = Item(name='item2', uid='2', quantity=quantity)
        with self.assertRaises(KeyError):
            item1.add(item2)

    def test_use_type(self):
        """Use returns Item"""
        self.assertIsInstance(
            Item(name='item', uid='1', quantity=2).use(1),
            Item
        )

    def test_use_result(self):
        """Use different quantities of item"""
        quantity_init = 2
        for quantity_use in range(0, quantity_init+1):
            with self.subTest(quantity_use=quantity_use):
                item = Item(name='item', uid='1', quantity=quantity_init)
                item_new = item.use(quantity_use)
                self.assertEqual(item.get_quantity(), quantity_init-quantity_use) # used item
                self.assertEqual(item_new.get_quantity(), quantity_use) # retreived item
        # remove too much
        quantity_use = quantity_init+2
        with self.subTest(quantity_use=quantity_use):
            item = Item(name='item', uid='1', quantity=quantity_init)
            item_new = item.use(quantity_use)
            self.assertEqual(item.get_quantity(), quantity_init) # used item
            self.assertEqual(item_new.get_quantity(), 0) # retreived item

class TestInventory(unittest.TestCase):
    """Test ModuleInventory from inventory.py
    """

    def test_iter(self):
        """Iterate over inventory yields all item"""
        inv = ModuleInventory()
        items = [
            Item(name='item1', uid='1'),
            Item(name='item2', uid='2')
        ]
        for i in items:
            inv.add(i)
        new_item_list = list()
        for i in inv.get_items():
            new_item_list.append(i)
        self.assertEqual(items, new_item_list)

    def test_add(self):
        """Add item quantity to new item within inventory"""
        quantity = 1
        item1 = Item(name='item', uid='1', quantity=quantity)
        item2 = Item(name='item', uid='1', quantity=quantity)
        inv = ModuleInventory()
        inv.add(item1)
        inv.add(item2)
        item_new = inv.get('1')
        self.assertEqual(item_new.get_quantity(), quantity+quantity)
        self.assertEqual(item1.get_quantity(), quantity+quantity)
        self.assertEqual(item2.get_quantity(), quantity-quantity)

    def test_use_type(self):
        """Use returns Item"""
        inv = ModuleInventory()
        item_id = '1'
        inv.add(Item(name='item', uid=item_id, quantity=2))
        self.assertIsInstance(
            inv.use(item_id=item_id, quantity=1),
            Item
        )      

    def test_use(self):
        """Use different quantities of item"""
        item_id = '1'
        quantity_init = 2
        # dont remove negative values
        for quantity_use in range(-2, 0):
            with self.subTest(quantity_use=quantity_use):
                inv = ModuleInventory()
                inv.add(Item(name='item', uid=item_id, quantity=quantity_init))
                with self.assertRaises(AssertionError):
                    inv.use(item_id=item_id, quantity=quantity_use, partial=False)
        # good case
        for quantity_use in range(0, quantity_init+1):
            with self.subTest(quantity_use=quantity_use):
                inv = ModuleInventory()
                inv.add(Item(name='item', uid=item_id, quantity=quantity_init))
                inv.use(item_id=item_id, quantity=quantity_use, partial=False)
                self.assertEqual(inv.get(item_id).get_quantity(), quantity_init-quantity_use)
        # remove too much should remove nothing -> quantity_init remains
        for quantity_use in range(quantity_init+2, quantity_init+5):
            with self.subTest(quantity_use=quantity_use):
                inv = ModuleInventory()
                inv.add(Item(name='item', uid=item_id, quantity=quantity_init))
                inv.use(item_id=item_id, quantity=quantity_use, partial=False)
                self.assertEqual(inv.get(item_id).get_quantity(), quantity_init)

    def test_use_partial(self):
        """Use different quantities of item"""
        item_id = '1'
        quantity_init = 2
        # dont remove negative values
        for quantity_use in range(-2, 0):
            with self.subTest(quantity_use=quantity_use):
                inv = ModuleInventory()
                inv.add(Item(name='item', uid=item_id, quantity=quantity_init))
                with self.assertRaises(AssertionError):
                    inv.use(item_id=item_id, quantity=quantity_use, partial=False)
        # good case
        for quantity_use in range(0, quantity_init+1):
            with self.subTest(quantity_use=quantity_use):
                inv = ModuleInventory()
                inv.add(Item(name='item', uid=item_id, quantity=quantity_init))
                inv.use(item_id=item_id, quantity=quantity_use, partial=True)
                self.assertEqual(inv.get(item_id).get_quantity(), quantity_init-quantity_use)
        # remove too much should remove all available
        for quantity_use in range(quantity_init+2, quantity_init+5):
            with self.subTest(quantity_use=quantity_use):
                inv = ModuleInventory()
                inv.add(Item(name='item', uid=item_id, quantity=quantity_init))
                new_item = inv.use(item_id=item_id, quantity=quantity_use, partial=True)
                self.assertEqual(new_item.get_quantity(), quantity_init)  # test remainder
                self.assertEqual(inv.get(item_id).get_quantity(), 0)  # test remainder

    def test_remove(self):
        """After remove item_id shall be gone"""
        inv = ModuleInventory()
        item_id = '1'
        inv.add(Item(name='item', uid=item_id, quantity=2))
        inv.remove(item_id)
        with self.assertRaises(KeyError):
            inv.get(item_id)

    def test_clear(self):
        """After clear() the inventory should be empty"""
        inv = ModuleInventory()
        inv.add(Item(name='item', uid='1'))
        inv.clear()
        self.assertDictEqual(inv.items, {}) # empty dict
        
    def test_to_msg(self):
        inv = ModuleInventory()
        item = Item(name='item', uid='1')
        inv.add(item)
        inventory_msg = inv.to_msg()
        self.assertIsInstance(inventory_msg, passform_msgs.msg.Inventory) # type of message
        self.assertEqual(Item.from_msg(inventory_msg.data[0]), item) # content of first message is input

    def test_storage(self):
        inv = ModuleInventory()        
        uid = str(uuid.uuid4())
        topic = str(uuid.uuid4())
        location = passform_msgs.msg.Location(
            aoi = passform_msgs.msg.AreaOfInterest(uuid=uid)
        )
        item = Item(name='item', uid='1', location=location)
        inv.add_storage(location=location, driver_topic=topic)
        new_item = inv.add(item)
        self.assertEqual(new_item.get_location().to_msg(), item.get_location().to_msg())
        self.assertEqual(inv.get_storage('1').location.to_msg(), location)
        self.assertEqual(inv.get_storage('1').driver_topic, topic)

    def test_storage_missing(self):
        """ValueError if storage is given but not existing"""
        inv = ModuleInventory()
        item = Item(
            name='item',
            uid='1',
            location= passform_msgs.msg.Location(
                aoi = passform_msgs.msg.AreaOfInterest(uuid='test')
            )
        )
        with self.assertRaises(ValueError):
            inv.add(item)

    def test_storage_occupied(self):
        """ValueError if storage is given but not existing"""
        inv = ModuleInventory()        
        uid = str(uuid.uuid4())
        topic = str(uuid.uuid4())
        location = passform_msgs.msg.Location(
            aoi = passform_msgs.msg.AreaOfInterest(uuid=uid)
        )
        inv.add_storage(location=location, driver_topic=topic)
        storage = Storage(location=location, driver_topic=topic)
        inv.add(Item(name='item', uid='1',location=location))
        with self.assertRaises(ValueError):
            inv.add(Item(name='item2', uid='2',location=location))

    def test_storage_clear(self):
        """clear should remove an item from storage"""
        inv = ModuleInventory()
        uid = str(uuid.uuid4())
        topic = str(uuid.uuid4())
        location = passform_msgs.msg.Location(
            aoi = passform_msgs.msg.AreaOfInterest(uuid=uid)
        )
        inv.add_storage(location=location, driver_topic=topic)
        inv.add(Item(name='item', uid='1', location=location))
        inv.clear()
        inv.add(Item(name='item2', uid='2', location=location))

    def test_storage_remove(self):
        """remove should remove an item from storage"""
        inv = ModuleInventory()
        item = Item(name='item', uid='1')
        uid = str(uuid.uuid4())
        topic = str(uuid.uuid4())
        location = passform_msgs.msg.Location(
            aoi = passform_msgs.msg.AreaOfInterest(uuid=uid)
        )
        inv.add_storage(location=location, driver_topic=topic)
        storage = Storage(location=location, driver_topic=topic)
        inv.add(Item(name='item', uid='1', location=location))
        inv.remove('1')
        inv.add(Item(name='item2', uid='2', location=location))

if __name__ == '__main__':
    unittest.main()