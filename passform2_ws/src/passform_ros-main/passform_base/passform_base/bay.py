from typing import Optional

from basyx.aas import model # Import all BaSyx Python SDK classes from the model package
from basyx.aas.model import base  # Import identically to basyx submodel.py

from passform_util.watchdog import Watchdog
from passform_util.basyx.rest_server import RestServer

from passform_util.types import Bay
from passform_util.types.status import ConnectionStatus

from passform_msgs.msg import Supply

class ActiveBay(Bay):
    def __init__(self,
        unique_id: str,
        name: str,
        origin: list=[0.0,0.0,0.0],
        parent: model.Identifiable = None,
        description: Optional[base.LangStringSet] = None,
        **kwargs
    ):
        self.watchdog = None
        super().__init__(
            unique_id = unique_id,
            name = name,
            origin = origin,
            **kwargs)
        self.server = RestServer()
        self.init_occupation(kwargs)

    def register(self, module_uuid, timeout_handler:callable=None) -> None:
        """
        store module uuid and set status to connected. start watchdog.
        """
        if not self.is_free():
            raise RuntimeError(f'Bay {self.get_id()} already in used.')
        self.occupy(module_uuid)
        self.timeout_handler = timeout_handler
        # handler = timeout_handler if timeout_handler is not None else self.timeout_handler
        self.watchdog = Watchdog(2, self.timeout_cb)

    def set_free(self):
        """Remove module and cancel timeout"""
        if self.watchdog is not None:
            self.watchdog.stop()
        self.watchdog = None
        self.set_timeout(False)
        self.set_module('')

    def is_free(self) -> bool:
        return self.get_module() == ''

    def is_virtual(self) -> bool:
        """Virtual bays have not physical representation. used for e.g. operators"""
        for obj in self.submodel_element:
            if obj.id_short == 'virtual':
                return obj.value
        return False

    def get_id(self) -> int:
        return int(self.id_short.split('bay')[1])

    def get_supply(self) -> Supply:
        msg = Supply()
        for data in self.submodel_element:
            if data.id_short == 'SupplyStatus':
                for obj in data:
                    if obj.id_short == 'dc_24v':
                        msg.dc_24v = obj.value
                    if obj.id_short == 'ac_230v':
                        msg.ac_230v = obj.value
                    if obj.id_short == 'vacuum':
                        msg.vacuum = obj.value
        return msg

    def occupy(self, module_uuid:str):
        """Add a module to the bay. Reset timeout"""
        self.set_timeout(False)
        self.set_module(module_uuid)

    def set_timeout(self, value:bool):
        """Set basyx value 'timeout'"""
        for obj in self.submodel_element:
            if obj.id_short == 'timeout':
                obj.value = value
        self.set_value(
            value_idshort='timeout',
            value=value
        )

    def get_timeout(self) -> bool:
        """Get basyx value 'timeout'"""
        for obj in self.submodel_element:
            if obj.id_short == 'timeout':
                return obj.value

    def set_module(self, value:str):
        """Set basyx value 'module'"""
        for obj in self.submodel_element:
            if obj.id_short == 'module':
                obj.value = value
        self.set_value(
            value_idshort='module',
            value=value
        )

    def get_module(self) -> str:
        """Get module uid from basyx"""
        for obj in self.submodel_element:
            if obj.id_short == 'module':
                return obj.value

    def get_connection_status(self) -> ConnectionStatus:
        if not self.is_free():
            if self.get_timeout():
                return ConnectionStatus.TIMEOUT
            return ConnectionStatus.CONNECTED
        else:
            return ConnectionStatus.DISCONNECTED

    def init_occupation(self, kwargs) -> None:
        """Init all elements describing the occupation status.

        Modifies `module`, `timeout`, `virtual`
        """
        self.submodel_element.add(
            value = model.Property(
                id_short='module',
                value_type=model.datatypes.String,
                value='',
                description={
                    'en-us':'UUID of the module in this bay.',
                    'de':'UUID des Moduls in der Bucht.'
                }
            )
        )
        self.submodel_element.add(
            value = model.Property(
                id_short='timeout',
                value_type=model.datatypes.Boolean,
                value=False,
                description={
                    'en-us':'Indicates if the bay module has a timeout.',
                    'de':'Gibt an, ob das Modul der Bucht einen Timeout hat.'
                }
            )
        )
        if 'virtual' in kwargs:
            self.submodel_element.add(
                value = model.Property(
                    id_short='virtual',
                    value_type=model.datatypes.Boolean,
                    value = True,
                    description={
                        'en-us':'If true, this bay has no hardware representation but manages a non-standard module.',
                        'de':'Gibt an, ob die Bucht kein Hardware-Äquivalent hat, sondern ein nicht-standard Modul verwaltet.'
                    }
                )
            )

    def timeout_cb(self) -> None:
        """ Will call the internal timeout_handler() function """
        self.set_timeout(True)
        if self.timeout_handler is not None:
            self.timeout_handler(self.get_module())  # perform external timeout handler
            return
        raise RuntimeError(f'Module {self.get_module()} timed out.')

    def set_value(self, value_idshort, value):
        """Value setter function"""
        self.server.set_submodel_value(self,
            value_idshort=value_idshort,
            value=str(value).lower()
        )

if __name__ == '__main__':
    b = ActiveBay('id', 'bay1', virtual=True)
    print(b.is_free())
    # b.register('1')
    # print(f'Module {b.get_module()}')
    # print(b.is_free())
