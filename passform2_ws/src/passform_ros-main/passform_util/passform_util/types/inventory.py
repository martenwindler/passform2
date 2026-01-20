import copy

import passform_msgs.msg
from basyx.aas import model
from passform_util.types.location import Location
from passform_util.basyx import sanitize_basyx_string
from passform_util.types.parameter import get_parameter

class Item(model.SubmodelElementCollectionUnordered):
    """Collection to describe an item
    """
    def __init__(
        self,
        name : str,
        uid : str,
        quantity : int=0,
        location: Location | passform_msgs.msg.Location = passform_msgs.msg.Location(),
        **kwargs
    ):
        assert uid != '', 'unique id must not be empty'
        super().__init__(
            id_short='item_'+sanitize_basyx_string(str(uid)),   # needs some leading letters. Underscore is important for get_name()
            category='Item',
            **kwargs
        )
        self.value.add(
            model.Property(
                id_short='name',
                value_type=model.datatypes.String,
                value=str(name)
            )
        )
        self.value.add(
            model.Property(
                id_short='uid',
                value_type=model.datatypes.String,
                value=str(uid)
            )
        )
        self.set_quantity(quantity)
        self.set_location(location)
        
    def __eq__(self, other):
        """Compare ID and name of two items"""
        return all([
            self.get_name() == other.get_name(),
            self.get_uid() == other.get_uid(),
            # self.get_location() == other.get_location(),
            # self.get_quantity() == other.get_quantity(),
        ])

    def add(self, other: 'Item') -> None:
        """Merge self with another item if name and id match.

        Will sum the quantity.
        :param other: Item to merge with self
        """
        if other.id_short != self.id_short:
            raise KeyError(
                f'Adding quantity to not matching item names: {other.get_name()} to {self.get_name()}')
        self.set_quantity(self.get_quantity() + other.get_quantity())
        other.use(other.get_quantity())

    def use(self, quantity: int) -> 'Item':
        """Removes quantity from quantity.

        If number of requested items is larged than quantity, nothing will be removed.
        :param quantity: number to be removed.
        :returns: Deep copy of item with the quantity used.
        """        
        new_item = self.get_copy(quantity)
        self.set_quantity(self.get_quantity() - new_item.get_quantity())
        return new_item

    def get_copy(self, quantity: int) -> 'Item':
        """Returns a copy with quantity, if available.

        If number of requested items is larged than quantity, nothing will be removed.
        :param quantity: number to be removed.
        :returns: Deep copy of item with the quantity used.
        """
        assert quantity>=0, '"quantity" must be non-negative'
        if quantity > self.get_quantity():
            quantity = 0
        new_item = copy.deepcopy(self)
        new_item.set_quantity(quantity)
        return new_item

    def get_name(self) -> int:
        """Getter for Property name"""
        return get_parameter(self, 'name').value

    def get_uid(self) -> int:
        """Getter for Property unique id
        """
        return get_parameter(self, 'uid').value

    def set_quantity(self, quantity: int) -> None:
        """Setter for Property quantity"""
        for val in self.value:
            if val.id_short == 'quantity':
                val.value = quantity
                return
        self.value.add(
            model.Property(
                id_short='quantity',
                value_type=model.datatypes.Integer,
                value=quantity,
            )
        )
        assert self.get_quantity()>=0, '"quantity" must be non-negative'

    def get_quantity(self) -> int:
        """Getter for Property quantity"""
        return get_parameter(self, 'quantity').value
        
    def set_location(self, location: passform_msgs.msg.Location|Location) -> None:
        """Setter for Location quantity"""
        if location is None:
            return
        self._location = Location(location) # store as ModelElement, not msg
        if isinstance(location, Location):
            location = location.to_msg()
        for val in self.value:
            if val.id_short == 'location':
                val = Location(location)
                return
        self.value.add( Location(location) )
        
    def get_location(self) -> Location:
        """Getter for Location"""
        # return get_parameter(self, 'location')
        return self._location
        
    def has_location(self) -> bool:
        """True if an aoi with unique id exists"""
        return self.get_location().get_aoi().get_uid() != ''

    def to_msg(self) -> passform_msgs.msg.Item:
        """Create a ROS msg of the item"""
        msg = passform_msgs.msg.Item(
            part=passform_msgs.msg.Part(
                uuid=self.get_uid(),
                name=self.get_name(),
            ),
            location=self.get_location().to_msg(),
            quantity=self.get_quantity(),
            uuid=self.get_uid()
        )
        return msg

    @classmethod
    def from_msg(cls, msg: passform_msgs.msg.Item):
        """
        Create an instance of :class:`Item` from an item message.

        :param msg: An instance of :class:`passform_msgs.msg.Item`.
        """
        if not isinstance(msg, passform_msgs.msg.Item):
            raise TypeError('Must pass a passform_msgs.msg.Item object')
        return cls(
            name = msg.part.name,
            uid=msg.part.uuid,
            quantity=msg.quantity,
            location=msg.location
        )