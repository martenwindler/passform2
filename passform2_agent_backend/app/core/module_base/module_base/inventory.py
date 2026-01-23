#!/usr/bin/python3

from dataclasses import dataclass, field

import passform_msgs.msg
from passform_util.types import Item, Location

@dataclass(unsafe_hash=True)
class Storage:
    """simple dataclass to gather data for a storage location
    """
    driver_topic: str=field(hash=False)
    location: Location | passform_msgs.msg.Location = field(hash=False)
    _uid: str = field(default='')

    def __post_init__(self):
        if isinstance(self.location, passform_msgs.msg.Location):
            self.location = Location(self.location)
        self._uid = self.location.get_aoi().get_uid()
        assert self._uid != '', 'storage locaton needs a uid'

    def __eq__(self, other):
        return self.get_uid() == other.get_uid()

    def get_uid(self) -> str:
        """Get label of AOI"""
        return self.location.get_aoi().get_uid()

    def to_msg(self) -> passform_msgs.msg.Item:
        """Create passform_msgs.msg.Item ROS message""" 
        return passform_msgs.msg.Item(
            location = self.location.to_msg()
        )

class ModuleInventory:
    """Manages a list to store Items and their quantity
    """

    def __init__(self):
        self.items: dict[str, Item] = dict()
        self.locations: dict[Storage, str] = dict()

    def __iter__(self):
        for item in self.items.values():
            yield item

    def get_ids(self) -> list[str]:
        """Get uid of all stored items"""
        return [item.get_uid() for item in self.get_items()]

    def get(self, item_id: str) -> Item:
        """Get Item with item_id. Returns None if not found.

        :param item_id: id of the Item to be retreived.
        :raises KeyError: if item is not found.
        """
        if item_id not in self.get_ids():
            raise KeyError('Item not found')
        return self.items[item_id]

    def get_items(self) -> list[Item]:
        """Get all items in storage"""
        return [item for item in self.items.values()]

    def add_storage(self, location: passform_msgs.msg.Location|Location, driver_topic: str):
        """Add an available storage location"""
        storage = Storage(location=location,driver_topic=driver_topic)
        if self.storage_exists(storage):
            raise KeyError(f'Cant add strage {storage.get_uid()}. It`s already there')
        self.locations[storage] = ''

    def get_storage(self, item_id: str) -> Storage:
        """Get Storage location of a stored item"""
        for loc, item in self.locations.items():
            if item == item_id:
                return loc

    def get_storage_location(self) -> list[Storage]:
        """Get all available locations"""
        return self.locations.keys()

    def storage_exists(self, storage: Storage) -> bool:
        """Returns True, if storage label exists in local storage list"""
        if storage is None:
            return False
        return storage.get_uid() in [stor.get_uid() for stor in self.get_storage_location()]

    def is_empty(self, storage: Storage) -> bool:
        """Returns True, if storage is available"""
        return self.locations[storage] == ''

    def set_storage(self, item: Item, storage: Storage) -> None:
        """occupies the storage (by aoi.label) with the item uid.

        :param item: Item to be stored.
        """
        if not self.storage_exists(storage):
            raise ValueError(f'Storage {storage.get_uid()} does not exist')
        if not self.is_empty(storage):
            raise ValueError(f'Storage {storage.get_uid()} already occupied')
        self.locations[storage] = item.get_uid()
            

    def add(self, item: Item) -> Item:
        """Adds an Item to the inventory. Increases the Quantity if Item is already stored.

        :param item: Item to be stored.
        :returns: Newly stored or updated Item
        """
        if item.get_uid() in self.get_ids():
            self.get(item.get_uid()).add(item)
        else:
            if item.has_location():
                self.set_storage(item, Storage(
                    location=item.get_location().to_msg(),
                    driver_topic=''
                ))
            self.items[item.get_uid()] = item
        return self.get(item.get_uid())

    def remove(self, item_id: str) -> None:
        """Removes Item by id from inventory.

        :param item_id: id of the Item to be removed.
        """
        self.items.pop(item_id)
        for loc, item in self.locations.items():
            if item == item_id:
                self.locations[loc] = ''

    def use(self, item_id: str, quantity: int, partial: bool = False) -> Item:
        """Removes the specified quantity of Item by its id.

        :param item_id: id of the Item to be retreived.
        :param quantity: number of items to remove.
        :param partial: If True, items will also be used if not enough are available.
        :returns: 
        """
        item_old = self.get(item_id)
        if partial:
            quantity = min(quantity, item_old.get_quantity())
        return item_old.use(quantity)

    def available(self, item_id: str, quantity: int, partial: bool = False) -> Item:
        """Retreives a copy of the requested item without affecting storage.

        :param item_id: id of the Item to be retreived.
        :param quantity: number of items to check for.
        :param partial: If True, the max number of available items will be provided.
        :returns: 
        """
        item_old = self.get(item_id)
        if partial:
            quantity = min(quantity, item_old.get_quantity())
        return item_old.get_copy(quantity)

    def clear(self) -> None:
        """Remove all items"""
        self.items.clear()
        for loc in self.locations.keys():
            self.locations[loc] = ''

    def to_msg(self) -> passform_msgs.msg.Inventory:
        """Create a ROS msg of the entire inventory"""
        msg = passform_msgs.msg.Inventory(
            data=[item.to_msg() for item in self.get_items()]
        )
        # add empty storage locations
        for storage in self.get_storage_location():
            if self.is_empty(storage):
                msg.data.append(storage.to_msg() )
        return msg
