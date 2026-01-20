from basyx.aas import model
from basyx.aas.model import base
from passform_util.basyx import sanitize_basyx_string
from passform_util.watchdog import Watchdog


class BayAsset(model.Asset):
    def __init__(self,
        unique_id: str
        ):

        super().__init__(
            kind = model.base.AssetKind.INSTANCE,
            identification=model.Identifier(unique_id+'_ASSET', model.IdentifierType.CUSTOM),
            category='bay'
        )

class TechnicalData(model.Submodel):
    def __init__(
        self,
        origin:list,
        unique_id:str,
    ):
        super().__init__(
            identification=model.Identifier(unique_id+'_TD', model.IdentifierType.CUSTOM),
            id_short='TechnicalData',
        )
        self.init_geometry(origin)

    def init_geometry(self, origin:list):
        """
        :origin: list with x,y,z
        """
        assert len(origin) == 3, 'origin must be specified with three elements'

        collection = model.SubmodelElementCollectionUnordered(
            id_short='Origin',
            description={
                'en-us':'Description of the bays origin.',
                'de':'Beschreibung des Ursprungs der Bucht.'
            }
        )
        collection.value.add(
            model.Property(
                id_short='x',
                value_type=model.datatypes.Double,
                value=origin[0],
                description={
                    'en-us':'x-position of the origin in m.',
                    'de':'x-Position des Ursprungs in m.'
                }
            )
        )
        collection.value.add(
            model.Property(
                id_short='y',
                value_type=model.datatypes.Double,
                value=origin[1],
                description={
                    'en-us':'y-position of the origin in m.',
                    'de':'y-Position des Ursprungs in m.'
                }
            )
        )
        collection.value.add(
            model.Property(
                id_short='z',
                value_type=model.datatypes.Double,
                value=origin[2],
                description={
                    'en-us':'z-position of the origin in m.',
                    'de':'z-Position des Ursprungs in m.'
                }
            )
        )
        self.submodel_element.add(collection)

class BayStatus(model.Submodel):
    def __init__(
        self,
        unique_id: str,
    ):
        super().__init__(
            identification=model.Identifier(unique_id+'_STATUS', model.IdentifierType.CUSTOM),
            id_short='BayStatus',
        )
        self.init_supply()
        self.init_occupation()

    def free(self):
        self.set_occu(False)
        self.set_timeout(False)
        self.set_module('')

    def occupy(self, module_uuid:str):
        self.set_occu(True)
        self.set_timeout(False)
        self.set_module(module_uuid)

    def is_free(self):
        for obj in self.submodel_element:
            if obj.id_short == 'occupation':
                return not obj.value

    def set_occu(self, value:bool):
        for obj in self.submodel_element:
            if obj.id_short == 'occupation':
                obj.value = value

    def set_timeout(self, value:bool):
        for obj in self.submodel_element:
            if obj.id_short == 'timeout':
                obj.value = value

    def get_timeout(self) -> bool:
        for obj in self.submodel_element:
            if obj.id_short == 'timeout':
                return obj.value

    def set_module(self, value:str):
        for obj in self.submodel_element:
            if obj.id_short == 'module':
                obj.value = value

    def get_module(self) -> str:
        for obj in self.submodel_element:
            if obj.id_short == 'module':
                return obj.value

    def init_occupation(self):
        self.submodel_element.add(
            value = model.Property(
                id_short='occupation',
                value_type=model.datatypes.Boolean,
                value=False,
            )
        )
        self.submodel_element.add(
            value = model.Property(
                id_short='module',
                value_type=model.datatypes.String,
                value='',
            )
        )
        self.submodel_element.add(
            value = model.Property(
                id_short='timeout',
                value_type=model.datatypes.Boolean,
                value=False,
            )
        )

    def init_supply(self):
        collection = model.SubmodelElementCollectionUnordered(
            id_short='SupplyStatus',
            description={
                'en-us':'List of all utilities currently active.',
                'de':'Liste aller aktuell aktiven Versorgungen.'
            }
        )
        collection.value.add(
            model.Property(
                id_short='dc_24v',
                value_type=model.datatypes.Boolean,
                value=True,
                description={
                    'en-us':'24V DC.',
                    'de':'24V DC.'
                }
            )
        )
        collection.value.add(
            model.Property(
                id_short='ac_230v',
                value_type=model.datatypes.Boolean,
                value=False,
                description={
                    'en-us':'230V AC.',
                    'de':'230V AC.'
                }
            )
        )
        self.submodel_element.add(collection)

class Bay(model.AssetAdministrationShell):
    def __init__(self,
        unique_id: str,
        name: str,
        origin: list=[0.0,0.0,0.0],
        parent: model.Identifiable = None,
        description: Optional[base.LangStringSet] = None,
        # properties: List[model.Property] = field(default_factory=list, compare=False),
        **kwargs
    ):
        self.watchdog = None
        super().__init__(
            identification=model.Identifier(unique_id, model.IdentifierType.CUSTOM),
            id_short=sanitize_basyx_string(name), #replace ' ', '-'
            category='passform_bay',
            submodel={TechnicalData(origin, unique_id),BayStatus(unique_id)},
            asset=BayAsset(unique_id),
            parent=parent,
            **kwargs)

        self.free()

    def get_id(self):
        return self.identification

    def get_module(self) -> str:
        return self.get_status().get_module()

    def register(self, module_uuid, timeout_handler=None):
        """
        store module uuid and set status to connected. start watchdog.
        """
        if not self.is_free():
            raise RuntimeError(f'Bay {self.get_id()} already in used.')
        self.get_status().occupy(module_uuid)
        self.timeout_handler = timeout_handler
        # handler = timeout_handler if timeout_handler is not None else self.timeout_cb
        self.watchdog = Watchdog(2, self.timeout_cb)

    def free(self):
        if self.watchdog is not None:
            self.watchdog.stop()
        self.watchdog = None
        self.get_status().free()

    def is_free(self):
        return self.get_status().is_free()

    def get_status(self):
        for s in self.submodel:
            if s.id_short == 'BayStatus':
                return s

    def timeout_cb(self):
        self.get_status().set_timeout(True)
        if self.timeout_handler is not None:
            self.timeout_handler()  # perform external timeout handler
            return
        raise RuntimeError(f'Module {self.get_module()} timed out.')


if __name__ == '__main__':
    from passform_util.basyx.rest_server import RestServer

    b = Bay('id', 'bay1')
    print(b.is_free())
    b.register('1')
    print(b.is_free())
    # b.free()
    # print(b.is_free())
    # for sm in b.submodel:
    #     print(sm)
    #     for e in sm.submodel_element:
    #         print(e)
