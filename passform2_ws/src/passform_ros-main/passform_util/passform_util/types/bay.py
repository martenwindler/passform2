from typing import Optional

from basyx.aas import model
from basyx.aas.model import base
from passform_util.basyx import sanitize_basyx_string


class Bay(model.Submodel):
    def __init__(self,
        unique_id: str,
        name: str,
        origin: list=[0.0,0.0,0.0],
        parent: model.Identifiable = None,
        description: Optional[base.LangStringSet] = None,
        virtual: bool = False,
        # properties: List[model.Property] = field(default_factory=list, compare=False),
        **kwargs
    ):
        super().__init__(
            identification=model.Identifier(unique_id, model.IdentifierType.CUSTOM),
            id_short=sanitize_basyx_string(name), #replace ' ', '-'
            category = 'passform_bay' if not virtual else 'virtual_bay',
            parent=parent,
            **kwargs)

        if not virtual:
            self.init_geometry(origin)
            self.init_supply()

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
