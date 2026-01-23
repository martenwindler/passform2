#!/usr/bin/env python3

import requests
import json

from basyx.aas import model, backend, adapter
from basyx.aas.model import base
import basyx.aas.adapter.json as basyx_json

import passform_util.network
from passform_util.basyx import sanitize_id, response_to_exception
from passform_util.basyx.rest_registry import RestRegistry
import passform_util.basyx.deserialize as deserialize

class RestServer:
    """
    Class to manage basic REST API calls to a BaSyx AAS Server

    API can be found at:
    https://app.swaggerhub.com/apis/BaSyx/basyx_asset_administration_shell_repository_http_rest_api/v1
    """

    def __init__(
        self,
        host: str=None,
        port: int=8081,
        registry: str='localhost',
        timeout: float|int = 3 # REST request timeout (used for both server and registry)
    ):
        # host = passform_util.network.get_ip()
        host = registry.split(':')[0]
        if not(passform_util.network.is_port_in_use(host=host,port=port)):
            print(f'Warning: Server port {host}:{port} not active.')
            if not(passform_util.network.ip_reachable(host=host)):
                print(f'Warning: Host {host} not reachable.')
        self._timeout = float(timeout)
        self.port = port
        self.api_url = ':'+str(self.port)+'/aasServer/shells/'
        self.host = 'http://'+host
        self.aas_server = self.host + self.api_url
        self.registry = RestRegistry(registry, timeout=self._timeout)

    def add_aas(self, aas_object: model.AssetAdministrationShell) -> None:
        """
        Adds AAS to the AAS Server via REST call.
        All assets and submodels referenced in the AAS have to be stored in the object.

        :aas_object: AAS object with all data stored internally
        """
        try:
            aas, submodel_objects = deserialize.seperate_submodel_from_aas(aas_object)
            self.add_shell(aas)
            for s in submodel_objects:
                self.add_submodel(aas_object.identification.id, s)
        except Exception as e:
            raise RuntimeError(f'REST PUT of AAS "{aas_object.identification.id}" failed. Error: {e}.')

    def remove_aas(self, aas_id:str) -> bool:
        """
        Removes the AAS with the specified unique ID via REST API delete

        :aas_id: ['identification']['id'] of the AAS. Will be sanitized
        :return: True if API responses with success (status 200)
        """
        r = requests.delete(
            self.aas_server+sanitize_id(aas_id),
            timeout = self._timeout
        )
        if r.status_code == 200: # 200 is success
            return self.registry.remove_aas(aas_id)
        else:
            return False

    def remove_submodel(self, aas_id:str, submodel_idshort:str) -> bool:
        """
        Removes the submodel with the specified unique ID via REST API delete

        :aas_id: ['identification']['id'] of the AAS. Will be sanitized
        :submodel_idshort: ['idShort'] of the submodel
        :return: True if API responses with success (status 200)
        """
        aas_id = sanitize_id(aas_id)
        submodel_idshort = sanitize_id(submodel_idshort)
        submodel_id = self.get_submodel(aas_id,submodel_idshort).identification.id
        r = requests.delete(
            self.aas_server+aas_id+'/aas/submodels/'+submodel_idshort,
            timeout = self._timeout
        )
        if r.status_code == 200: # 200 is success
            return self.registry.remove_submodel(aas_id, submodel_id)
        else:
            raise RuntimeError(f'REST DELETE failed. {response_to_exception(r)}')

    def get_aas_id(self) -> list[str]:
        """Get the identification ID of all AAS on the server"""
        shells = requests.get(self.aas_server, timeout=self._timeout).json()
        ids = [n['identification']['id'] for n in shells]
        return ids

    def get_dynamic_aas(self, aas_id) -> model.AssetAdministrationShell:
        """
        Retreives JSON data from REST and turns into AAS object

        WARNIGN: The performed casts are very un-nice.The reason is a problem with decoding
        internally stored data. Somehow, only references are used nicely.

        :aas_id: ['identification']['id'] of the AAS. Will be sanitized
        """
        # get aas
        r = requests.get(self.aas_server+aas_id+'/aas/', timeout=self._timeout)
        if r.status_code != 200:
            raise RuntimeError(f'Failed to get submodel. {response_to_exception(r)}')
        aas_json = r.json()
        # get submodel
        r = requests.get(
            self.aas_server+aas_id+'/aas/submodels/',
            timeout = self._timeout
        )
        if r.status_code != 200:
            raise RuntimeError(f'Failed to get submodel. {response_to_exception(r)}')
        submodels_json = r.json()
        # create submodel objects for replacement later on
        submodel_obj = set()
        for s in submodels_json:
            submodel_obj.add(json.loads(json.dumps(s), cls=basyx_json.AASFromJsonDecoder))
        # decode json data to AAS obect
        aas_obj = deserialize.deserialize_json_aas(aas_json)
        return aas_obj

    def get_aas(self, aas_id) -> dict:
        """
        Retreives JSON data from REST and turns into AAS object

        Only uses the shell information an no specific submodel calls.
        :aas_id: ['identification']['id'] of the AAS. Will be sanitized
        """
        r = requests.get(self.aas_server+aas_id+'/aas/',timeout=self._timeout)
        if r.status_code != 200:
            raise RuntimeError(f'Failed to get AAS. {response_to_exception(r)}')
        aas_json = r.json()
        # try to get submodels
        r = requests.get(
            self.aas_server+aas_id+'/aas/submodels/',
            timeout = self._timeout
        )
        if r.status_code == 200:
            submodels = r.json()
            if len(submodels) > 0:
                aas_json['submodels'] = submodels
        return deserialize.deserialize_json_aas(aas_json)

    def get_submodel(self, aas_id:str, submodel_idshort:str) -> model.Submodel:
        """
        Retreives a submodel from the server.

        Both IDs will be sanitized.
        :aas_id: ['identification']['id'] of the AAS. Will be sanitized
        :submodel_idshort: ['idShort'] of the submodel
        """
        aas_id = sanitize_id(aas_id)
        submodel_idshort = sanitize_id(submodel_idshort)
        r = requests.get(
            self.aas_server+aas_id+'/aas/submodels/'+submodel_idshort+'/submodel',
            timeout = self._timeout
        )
        if r.status_code != 200:
            raise RuntimeError(f'Failed to get submodel "{submodel_idshort}". {response_to_exception(r)}')
        return deserialize.deserialize_json_submodel(r.json())

    def add_shell(self, aas_object: model.AssetAdministrationShell):
        """
        Adds a shell object to the AAS Server via REST call.
        Will not add submodels to the server.

        :aas_object: shell object
        """
        data_json = json.loads(json.dumps(aas_object, cls=basyx_json.AASToJsonEncoder))
        # basyx_json.AASToJsonEncoder._asset_administration_shell_to_json(aas_object)
        aas_id_clean = sanitize_id(data_json['identification']['id'])
        # if 'submodels' in data_json.keys():
        #     del data_json['submodels']

        r = requests.put(
            self.aas_server+aas_id_clean,
            data=str(json.dumps(data_json,cls=basyx_json.AASToJsonEncoder)),
            timeout = self._timeout
        )
        if r.status_code == 200:
            # 200 == success
            self.registry.add_aas(aas_object, endpoint_url = self.host+self.api_url+aas_id_clean)
        else:
            raise RuntimeError(f'Failed to add AAS object "{aas_id_clean}". {response_to_exception(r)}')

    def add_submodel(self, aas_id:str, submodel:model.Submodel) -> None:
        """
        Adds Submodel to the AAS Server via REST call.
        WARNING: Nested submodels will not be transfered.

        :aas_id: ['identification']['id'] of the AAS. Will be sanitized
        :submodel: Submodel object with all data stored internally
        """
        data_json = json.loads(json.dumps(submodel,cls=basyx_json.AASToJsonEncoder))
        aas_id_clean = sanitize_id(aas_id)
        submodel_idshort_clean = data_json['idShort']
        if 'submodelElements' in data_json.keys():
            data_json['submodelElements'] = list(filter(lambda d: d['modelType']['name'] != 'Submodel', data_json['submodelElements']))
        r = requests.put(
            self.aas_server+aas_id_clean+'/aas/submodels/'+submodel_idshort_clean,
            data=json.dumps(data_json,cls=basyx_json.AASToJsonEncoder),
            timeout = self._timeout
        )
        if r.status_code == 200:
            # 200 == success
            self.registry.add_submodel(
                aas_id_clean, submodel,
                endpoint_url = self.host+self.api_url+aas_id_clean+'/aas/submodels/'+submodel_idshort_clean+'/submodel')
            for obj in self.find_problematic_elements(submodel):
                print(f'{aas_id} contained a nested submodel. Skipped {submodel.id_short}:{obj.id_short}.')
        else:
            raise RuntimeError(f'Failed to add submodel {submodel_idshort_clean} to {aas_id_clean}. {response_to_exception(r)}')

    def add_element(self, aas_id:str, submodel_id:str, obj: model.SubmodelElement) -> None:
        """
        Adds Element to Submodel of the AAS Server via REST call.

        :aas_id: ['identification']['id'] of the AAS. Will be sanitized
        :submodel: id_short of the submodel. Will be sanitized
        :param obj: SubmodelEment with all data stored internally
        """
        data_json = json.loads(json.dumps(obj,cls=basyx_json.AASToJsonEncoder))
        aas_id_clean = sanitize_id(aas_id)
        object_idshort_clean = data_json['idShort']
        r = requests.put(
            self.aas_server+aas_id_clean+'/aas/submodels/'+submodel_id+'/submodel/submodelElements/'+object_idshort_clean,
            data=json.dumps(data_json,cls=basyx_json.AASToJsonEncoder)
        )
        if r.status_code == 200:
            # 200 == success
            # TODO:
            # other adds have a registry modification here, but this currently seems not possible via REST.
            # API is listed here: https://app.swaggerhub.com/apis/BaSyx/BaSyx_Registry_API/v1
            return
        else:
            raise RuntimeError(f'Failed to add submodel element {object_idshort_clean} to '
                f'{aas_id_clean}\{submodel_id}. {response_to_exception(r)}')

    def set_value(
        self,
        aas_id: str,
        submodel_idshort: str,
        value_idshort: str,
        value: base.ValueDataType
    ) ->  None:
        """Set value of a single parameter. Will not change Registry."""
        a_id = sanitize_id(aas_id)
        s_id = sanitize_id(submodel_idshort)
        v_id = sanitize_id(value_idshort)
        data = json.dumps(value,cls=basyx_json.AASToJsonEncoder)
        r = requests.put(
            self.aas_server+a_id+'/aas/submodels/'+ \
            s_id+'/submodel/submodelElements/'+\
            v_id+'/value',
            data=data,
            timeout=self._timeout
        )
        if r.status_code !=200:
            raise RuntimeError(f'Failed to modify value "{v_id}" at {a_id}/{s_id}. {response_to_exception(r)}')

    def set_submodel_value(self, submodel:object, value_idshort:str, \
    value:base.ValueDataType) -> None:
        """Set value of a submodel parameter with known parent AAS. Will not change Registry."""
        parent = submodel.parent.key[0].value
        self.set_value(parent, submodel.id_short, value_idshort, value)

    def get_value(
        self,
        aas_id: str,
        submodel_idshort: str,
        value_idshort: str
    ) ->  base.ValueDataType:
        """Get value of a single parameter."""
        a_id = sanitize_id(aas_id)
        s_id = sanitize_id(submodel_idshort)
        v_id = sanitize_id(value_idshort)
        r = requests.get(
            self.aas_server+a_id+'/aas/submodels/'+ \
            s_id+'/submodel/submodelElements/'+\
            v_id+'/value',
            timeout=self._timeout
        )
        if r.status_code !=200:
            raise RuntimeError(f'Failed retreive value "{v_id}" from {a_id}/{s_id}. {response_to_exception(r)}')
        return r.json()

    def get_submodel_value(self, submodel:object, value_idshort:str) ->  base.ValueDataType:
        """Set value of a submodel parameter with known parent AAS. Will not change Registry."""
        parent = submodel.parent.key[0].value
        return self.get_value(parent, submodel.id_short, value_idshort)

    @staticmethod
    def find_problematic_elements(obj:model.Submodel) -> list:
        """
        Find objects in a model, that are unsupported.

        Currently only checks for nested submodels.
        :return: list of model objects that will not be added correctly to the REST server.
        """
        obj_list = list()
        if obj.__class__ == model.Submodel:
            # nested submodels are bad
            for e in obj.submodel_element:
                if e.__class__ == model.Submodel:
                    obj_list.append(e)
        return obj_list


if __name__ == '__main__':
    asset = model.Asset(
        id_short='ExampleAsset',
        kind=model.AssetKind.INSTANCE,
        identification=model.Identifier('https://acplt.org/Simple_Asset', model.IdentifierType.IRI)
    )
    p1 = model.Property(
        id_short='ExampleProperty',
        value_type=model.datatypes.String,
        value='exampleValue',
    )
    p2 = model.Range(
        id_short='ExampleRange',
        value_type=model.datatypes.Int,
        min=1,
        max=5,
    )
    sm1 = model.Submodel(
        id_short='layered_submodel',
        identification=model.Identifier('sm_layer1', model.IdentifierType.CUSTOM),
        submodel_element={p1}
    )
    sm2 = model.Submodel(
        id_short='layered_submodel_2',
        identification=model.Identifier('sm_layer2', model.IdentifierType.CUSTOM),
        submodel_element={p2}
    )
    aas = model.AssetAdministrationShell(
        id_short='ExampleAAS',
        identification=model.Identifier('AAS_Test', model.IdentifierType.CUSTOM),
        asset=asset,
        submodel={sm1}
    )

    # api
    api = RestServer()
    api.remove_aas(aas.identification.id)
    api.add_aas(aas)
    tmp = api.get_aas(aas.identification.id)

    ## Registry Bug
    for obj in tmp.submodel:
        api.set_submodel_value(obj, 'ExampleProperty', 'NewValue')
        value_server = api.get_submodel_value(obj, 'ExampleProperty')

    parameters = api.registry.get(f'http://localhost:8082/registry/api/v1/registry/{aas.identification.id}/submodels/{sm1.identification.id}')['submodelElements']
    for p in parameters:
        if p['idShort'] == 'ExampleProperty':
            value_registry = p['value']

    assert value_server == value_registry, f'{value_server} is not {value_registry}'
    ##

    # print( api.add_submodel('AAS_Test', submodel) )
    # aas_load = api.get_aas('AAS_Test')
    # submodel = api.get_submodel('AAS_Test','Submodel')
    # print(submodel)
    # print( api.remove_aas(aas.identification.id) )

    # ## asset
    # for s in ['kind', 'identification', 'id_short', 'category', 'description', 'parent', 'administration', 'asset_identification_model', 'bill_of_material']:
    #     print(s)
    #     print(getattr(aas.asset, s))
    #     print(getattr(aas_load.asset, s))

    # for m in aas_load.submodel:
    #     for se in m.submodel_element:
    #         print(se)
    #     for s in ['id_short', 'category', 'description', 'parent', 'administration', 'semantic_id', 'qualifier', 'kind']:
    #         print(s)
    #         print(getattr(m, s))
    #         # for e in m.submodel_element:
    #         #     for s1 in ['id_short', 'category', 'description', 'parent', 'semantic_id', 'qualifier', 'kind']:
    #         #         print(getattr(e, s1))
