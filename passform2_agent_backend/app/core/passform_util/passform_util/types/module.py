from dataclasses import dataclass, field
from typing import List, Dict, Optional
import time
import uuid

from basyx.aas import model # Import all BaSyx Python SDK classes from the model package
from basyx.aas.model import base  # Import identically to basyx submodel.py

from passform_util.types.status import Status, ConnectionStatus
import passform_msgs.msg
import diagnostic_msgs.msg

from passform_util.basyx import sanitize_basyx_string
from passform_util.basyx.conversion import dict_to_dataelement, data_to_blob

@dataclass
class Supply:
    dc_24v: bool = field(default=True)
    ac_230v: bool = field(default=False)
    vacuum: bool = field(default=False)

class ModuleAsset(model.Asset):
    def __init__(self,
        unique_id: str
        ):

        super().__init__(
            kind = model.base.AssetKind.INSTANCE,
            identification=model.Identifier(unique_id+'_ASSET', model.IdentifierType.CUSTOM),
            category='module_asset'
        )

class TechnicalData(model.Submodel):
    def __init__(self,
        unique_id: str,
        properties: dict = dict(),
        thumbnail: Optional[str] = None,
    ):
        super().__init__(
            identification=model.Identifier(unique_id+'_TD', model.IdentifierType.CUSTOM),
            id_short='TechnicalData',
        )
        properties_data = dict() if properties is None else properties
        self.init_data(properties_data, thumbnail)

    def init_data(self, properties, thumbnail=None):
        self.init_thumbnail(thumbnail)
        if 'TechnicalData' in properties:
            self.init_supply(properties['TechnicalData'])
            self.init_geometry(properties['TechnicalData'])

    def init_thumbnail(self,thumbnail):
        if thumbnail is None or thumbnail == '':
            return
        self.submodel_element.add(
            data_to_blob(
                data_path = thumbnail,
                id_short = 'thumbnail'
            )
        )

    def init_supply(self, properties:dict):
        if not 'Supply' in properties:
            return

        collection = model.SubmodelElementCollectionUnordered(
            id_short='SupplyCharacteristics',
            description={
                'en':'List of all utilities needed.',
                'de':'Sammlung aller benötigten Versorgungen.'
            }
        )
        if properties['Supply']['24vdc']:
            collection.value.add(
                model.Property(
                    id_short='dc_24v',
                    value_type=model.datatypes.Boolean,
                    value=True,
                    description={
                        'en':'The system requires a 24V DC supply.',
                        'de':'Das System benötigt eine 24V DC Versorgung.'
                    }
                )
            )
        if properties['Supply']['230vac']:
            collection.value.add(
                model.Property(
                    id_short='ac_230v',
                    value_type=model.datatypes.Boolean,
                    value=True,
                    description={
                        'en':'The system requires a 230V AC supply.',
                        'de':'Das System benötigt eine 230V AC Versorgung.'
                    }
                )
            )
        if properties['Supply']['vacuum']:
            collection.value.add(
                model.Property(
                    id_short='vacuum',
                    value_type=model.datatypes.Boolean,
                    value=True,
                    description={
                        'en':'The system requires a vacuum supply.',
                        'de':'Das System benötigt eine Vakuum Versorgung.'
                    }
                )
            )
        self.submodel_element.add(collection)

    def init_geometry(self, properties:dict):
        if not 'PhysicalProperties' in properties:
            return

        collection = model.SubmodelElementCollectionUnordered(
            id_short='PhysicalProperties',
            description={
                'en':'Collection of physical properties.',
                'de':'Sammlung physischer Eigenschaften.'
            }
        )
        for p in properties['PhysicalProperties']:
            try:
                obj = dict_to_dataelement(p)
                collection.value.add(obj)
            except Exception as e:
                print(e)
        self.submodel_element.add(collection)

class Inventory(model.Submodel):
    """Submodel to store dynamic inventory data. Used in Module AAS.
    
    Uses one `model.SubmodelElementCollectionUnordered` in which only Integer Properties should be
    stored.
    """
    def __init__(self,
        unique_id: str,
        data: Optional[dict] = None,
    ):
        super().__init__(
            identification=model.Identifier(unique_id+'_Inventory', model.IdentifierType.CUSTOM),
            id_short='inventory',
        )
        self.init_data(data)

    def init_data(self, data:dict):
        """Init inventory and populate with data"""

        collection = model.SubmodelElementCollectionUnordered(
            id_short='Inventory',
            description={
                'en':'Collection of items stored in the module.',
                'de':'Sammlung von im Modul gelagerten Gegenständen.'
            }
        )
        if data is None:
            return
        if not 'Inventory' in data:
            return
        for item in data['Inventory']:
            try:
                collection.value.add( dict_to_dataelement(item) )
            except Exception as err:
                print(err)
        self.submodel_element.add(collection)

class Module(model.AssetAdministrationShell):
    def __init__(self,
        unique_id: str,
        name: str,
        description: Optional[base.LangStringSet] = None,
        parent: Optional[str] = None,   # would need to be a reference
        properties: Optional[dict] = None,
        thumbnail: Optional[str] = None,
        **kwargs):

        super().__init__(
            identification=model.Identifier(unique_id, model.IdentifierType.CUSTOM),
            id_short=sanitize_basyx_string(name), #replace ' ', '-'
            category='module',
            submodel={TechnicalData(unique_id, properties, thumbnail), Inventory(unique_id, properties)},
            asset=ModuleAsset(unique_id),
            **kwargs)
        self.uuid = self.identification.id

if __name__ == '__main__':
    from passform_util.basyx.rest import RestServer
    import yaml
    # M = Module(name='Modulname', supplier_name='BIBA', uuid='13thjdaq', supply=Supply(dc_24v=True))
    # print(M.to_ros())
    with open('/home/jasper/git/passform_ros/passform_util/passform_util/types/config.yaml', 'r') as file:
        properties = yaml.safe_load(file)

    m = Module('uuid', 'name', properties = None, thumbnail='/home/jasper/git/passform_ros/passform_util/passform_util/basyx/lena.png')
    for obj in m.submodel:
        for o in obj:
            print(o)

    api = RestServer()
    api.remove_aas('uuid')
    api.add_aas(m)
    new = api.get_aas('uuid')
    for obj in new.submodel:
        for se in obj.submodel_element:
            for v in se.value:
                print(v)
                # print(f'{v.id_short}: {v.value}')

    # MI = ModuleInformation(uuid='13thjdaq', skills = [S1, S2])string
    # # Watchdog
    # MS = ModuleWatchdog(1, Status.WARN)
    # for i in range(0,30):
    #     MS.on_heartbeat();
    #     time.sleep(0.2)
    # # Print
    # print(M.to_ros())
    # print(MI.to_ros())
