import time
import uuid

from passform_base import ActiveBay, RegistrationRequest
from passform_util.basyx.connector import ConnectorFactory, RestServerConnector
from passform_util.types import Master, Module
from passform_util.types.status import ConnectionStatus


class DataManager:
    """
    This class handles the bay and module data on a high level (register modules, start
    watchdogs) and stores module data in a database.
    WARNING: This class does not update FROM basyx, therefore only locally stored data will be
    returned.
    """
    def __init__(
        self,
        registry_host: str
    ):
        self.aas = Master()
        self.basyx : RestServerConnector = self.init_basyx(registry_host)

    def init_bays(self, bay_id:list[int]):
        """ Initialize bay and module db. Deletes all old content """
        # bay data
        data = list()
        for id in bay_id:
            self.add_bay(RegistrationRequest(bay_id = id))

    def add_bay(self, request:RegistrationRequest) -> ActiveBay:
        """Create an ActiveBay object and adds it to basyx and local storage."""
        b = self.create_bay(request)
        self.basyx.add(obj=b, aas_id=self.aas.identification.id)
        # add AAS parent to local model
        # TODO: could be done internally without a dedicated GET from server
        tmp = self.basyx.get_submodel(self.aas.identification.id,b.id_short)
        b.parent = tmp.parent
        self.aas.submodel.add(b)
        return b

    def create_bay(self, request:RegistrationRequest) -> ActiveBay:
        """Create an ActiveBay object."""
        if request.is_virtual:
            b = ActiveBay(
                unique_id = str(uuid.uuid4()),
                name = f'VIRTUAL_{len(self.get_virtual_bays())}',
                virtual = True
            )
        else:
            b = ActiveBay(
                unique_id = str(uuid.uuid4()),
                name = f'Bay{request.bay_id}',
            )
        return b

    def add_module(self, module:Module, request:RegistrationRequest, timeout_handler=None) -> int:
        """
        Add module to list of active modules
        param timeout_handler : behavior if timeout is raised
        returns: the bay ID the module is registered in
        """
        # availablity assertions
        if self.is_registered(module.uuid):
            raise RuntimeError(f'Module {module.uuid} already registered.')
        # virtual bay
        if request.is_virtual:
            b = self.add_bay(request)
            b.register(module.uuid, timeout_handler)
            return 0
        # physical bays must have existing bay_id
        if not self.bay_exists(request.bay_id):
            raise RuntimeError(f'Bay {request.bay_id} does not exist.')
        # search bay and register
        for b in self.get_physical_bays():
            if b.get_id() == request.bay_id:
                b.register(module.uuid, timeout_handler)
        return request.bay_id

    def delete_module(self, uuid:str):
        """Delete module by uuid"""
        if not self.is_registered(uuid):
            raise RuntimeError('Module not registered.')
        for b in self.get_all_bays():
            if b.get_module() == uuid:
                self.free_bay(b)
                return

    def free_bay(self, bay:ActiveBay):
        """Calls the free() method of the bay. Removes bay, if it is virtual"""
        if bay.is_virtual():
            self.remove_bay(bay)
        else:
            bay.set_free()

    def remove_bay(self, bay:ActiveBay):
        """Remove a bay from basyx and local storage"""
        bay_id_short = bay.id_short
        self.basyx.discard(self.aas.identification.id, bay_id_short)
        for obj in self.aas.submodel:
            if obj.id_short == bay_id_short:
                self.aas.submodel.remove(obj)
                return

    def get_physical_bays(self) -> list[ActiveBay]:
        """Return the static bay list, not stored in Basyx
        Retreives only physical bays
        """
        return [b for b in self.aas.submodel if b.category == 'passform_bay']

    def get_virtual_bays(self) -> list[ActiveBay]:
        """Return the static bay list, not stored in Basyx
        Retreives only virtual bays
        """
        return [b for b in self.aas.submodel if b.category == 'virtual_bay']

    def get_all_bays(self) -> list[ActiveBay]:
        """Return the static bay list, not stored in Basyx
        Retreives both virtual and physical modules
        """
        return self.get_physical_bays() + self.get_virtual_bays()

    def reset_watchdog(self, uuid:str):
        """Resets the watchdog of a bay containing module with uuid"""
        for b in self.get_all_bays():
            if b.get_module() == uuid:
                try:
                    b.watchdog.reset()
                except AttributeError:
                    # catch "'NoneType' object has no attribute 'reset'" which happens if heartbeat
                    # is received before watchdog is started
                    pass

    def is_registered(self, uuid:str) -> bool:
        """Checks if the given module uuid is registered."""
        for b in self.get_all_bays():
            if b.get_module() == uuid:
                return True
        return False

    def bay_exists(self, bay_id:int) -> bool:
        """Checks if a specific bay exists."""
        for b in self.get_physical_bays():
            if b.get_id() == bay_id:
                return True
        return False

    def get_active_modules(self) -> list[str]:
        """Creates a list of all active module unique_ids"""
        return [b.get_module() for b in self.get_all_bays() if len(b.get_module())> 0]

    def get_module_count(self) -> int:
        """Gives the number of currently active modules"""
        return len(self.get_active_modules())

    def get_occuopied_bay_count(self) -> int:
        """Gives the number of currently occupied bays"""
        return len([b.get_module() for b in self.get_physical_bays() if len(b.get_module())> 0])

    def get_bay_count(self) -> int:
        """Gives the number of currently occupied bays"""
        return len(self.get_physical_bays())

    def init_basyx(self, registry_host:str) -> RestServerConnector:
        """Create RestServerConnector and add the internally stored AAS"""
        try:
            basyx = ConnectorFactory(config=registry_host)
            basyx.add(self.aas)
            return basyx
        except Exception as e:
            raise RuntimeError(f'Error initializing BaSyx. {e}')

def create_bay(id, idx=0):
    module_uuid = [
        'f858b300-51a6-49bc-b3b3-dd2ac4363ccd',
        '1610f08b-3995-47ff-95e8-664397a58606',
        '75247881-e263-4714-b4cb-bb9adc35f81a',
        '57af9632-718f-4831-b1dd-1de31e1a16f9',
        'f5f5342b-3660-4402-a681-94a3c024c309',
        'd35bde5a-51db-4385-bfa3-86791048659d',
        '8bcb061b-863f-49c4-850e-e997f7de1da3',
        '512a18c6-a3b9-4a06-a425-280fd6720561'
    ]
    return {
        'bay_id': id,
        'connection_status': ConnectionStatus.DISCONNECTED,
        'mac': 'none',
        'nfc_tag': 'none',
        'uuid': str(uuid.uuid4()),
        'module_uuid': '',
        # 'module_uuid': module_uuid[id-1],
        'supply': { 'dc_24v': True, 'ac_230v': False, 'vacuum': False }
      }

if __name__ == '__main__':
    # mm = DataManager(host='localhost', port=27017, db_name='passform', collection='bay')
    # mm.init_bays([1,2,3,4])
    m = Module(unique_id='1231-3123', name='test')
    # mm.add_module(m,1,print('timeout'))

    mm = DataManager(registry_host = '134.102.97.172:8082')
    mm.init_bays([1,2,3])
    print(mm.get_local_bays())
    mm.add_module(m,1,lambda:print('timeout'))
    time.sleep(20)
    mm.get_local_bays()
    # mm.add_module(Module(uuid='1231-3123', name='test'),1,print('timeout'))
    # print( mm.find_module({'uuid':'e06836bf-b86d-482e-bf5f-59f51aaec334'}) )
