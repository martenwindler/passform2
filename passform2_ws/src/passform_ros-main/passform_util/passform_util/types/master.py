import uuid

from basyx.aas import model
from basyx.aas.model import base
from passform_util.basyx import sanitize_basyx_string
from passform_util.types import Bay


class MasterAsset(model.Asset):
    def __init__(self,
        unique_id: str
        ):

        super().__init__(
            kind = model.base.AssetKind.INSTANCE,
            identification=model.Identifier(unique_id+'_ASSET', model.IdentifierType.CUSTOM),
            category='passform_master_asset'
        )

class TechnicalData(model.Submodel):
    def __init__(
        self,
        unique_id: str
    ):
        super().__init__(
            identification=model.Identifier(unique_id+'_TD', model.IdentifierType.CUSTOM),
            id_short='TechnicalData',
        )

class Master(model.AssetAdministrationShell):
    def __init__(self,
        unique_id: str = str(uuid.uuid4()),
        name: str = 'PassForM_Master',
        # properties: List[model.Property] = field(default_factory=list, compare=False),
        **kwargs):

        super().__init__(
            identification=model.Identifier(unique_id, model.IdentifierType.CUSTOM),
            id_short=sanitize_basyx_string(name), #replace ' ', '-'
            category='passform_master',
            submodel={TechnicalData(unique_id)},
            asset=MasterAsset(unique_id),
            **kwargs)

    def create_bay(self, bay_id, origin=[0.0,0.0,0.0]):
        b = Bay(str(uuid.uuid4()), 'Bay'+str(bay_id), origin=origin)
        self.submodel.add(b)

    def get_bays(self):
        resp = list()
        for s in self.submodel:
            if s.category == "passform_bay":
                resp.append(s)
        return resp

if __name__ == '__main__':
    import json
    import unittest

    import basyx.aas.adapter.json as basyx_json
    from passform_util.basyx.rest import RestServer

    m = Master('uuid')
    m.create_bay(1)
    m.create_bay(2)
    for b in m.get_bays():
        print(b)
    # data_in = json.loads(json.dumps(m, cls=basyx_json.AASToJsonEncoder))
    #
    # api = RestServer()
    # api.remove_aas('uuid')
    #
    # api.add_aas(m)
    # m2 = api.get_aas('uuid')
    # data_out = json.loads(json.dumps(m2, cls=basyx_json.AASToJsonEncoder))
    #
    # tc = unittest.TestCase()
    # tc.assertDictEqual(data_in, data_out)
