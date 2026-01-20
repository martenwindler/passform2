#!/usr/bin/env python3

import requests
import json

from basyx.aas import model, backend, adapter
import basyx.aas.adapter.json as basyx_json

import passform_util.network
from passform_util.basyx import split_host, sanitize_id, response_to_exception
import passform_util.basyx.deserialize as deserialize

class RestRegistry():
    """
    Class to manage basic REST API calls to a BaSyx Registry Server

    API can be found at:
    https://app.swaggerhub.com/apis/BaSyx/BaSyx_Registry_API/v1#

    :host: full url and port of the registry component (e.g. http://localhost:8082)
    """

    def __init__(
        self,
        host: str = 'localhost',
        port: int = 8082,
        timeout: float|int = 3 # REST request timeout
    ):
        host, port = split_host(host, port)
        if not(passform_util.network.is_port_in_use(host=host,port=port)):
            print(f'Warning: Registry port {host}:{port} not active.')
            if not(passform_util.network.ip_reachable(host=host)):
                print(f'Warning: Host {host} not reachable.')
        self.url = 'http://'+host+':'+str(port)+'/registry/api/v1/registry/'
        self._timeout = float(timeout)

    def add_aas(self, aas_object:model.AssetAdministrationShell, endpoint_url:str) -> bool:
        # create data
        data_json = json.loads(json.dumps(aas_object, cls=basyx_json.AASToJsonEncoder))
        data_json['modelType']['name'] = 'AssetAdministrationShellDescriptor'
        # if 'submodels' in data_json.keys():
        #     del data_json['submodels']
        data_json['endpoints'] = [{
            "address": endpoint_url,
            "type": "http",
            "parameters": {}
        }]
        aas_id=data_json['identification']['id']
        # send data
        r = requests.put(
            self.url+sanitize_id(aas_id),
            data = str(json.dumps(data_json,cls=basyx_json.AASToJsonEncoder)),
            timeout = self._timeout
        )
        if r.status_code != 200:
            raise RuntimeError(f'Failed to add to registry. {response_to_exception(r)}')
        # for s in aas_object.submodel:
        #     self.add_submodel(aas_id,s,'missing')

    def add_submodel(self, aas_id:str, submodel_object:model.Submodel, endpoint_url:str) -> bool:
        # create data
        data_json = basyx_json.AASToJsonEncoder._submodel_to_json(submodel_object)
        data_json['modelType']['name'] = 'SubmodelDescriptor'
        data_json['endpoints'] = [{
            "address": endpoint_url,
            "type": "http",
            "parameters": {}
        }]
        submodel_id_clean = sanitize_id(data_json['identification'].id)
        # send data
        r = requests.put(
            self.url+sanitize_id(aas_id)+'/submodels/'+submodel_id_clean,
            data = str(json.dumps(data_json,cls=basyx_json.AASToJsonEncoder)),
            timeout = self._timeout
        )
        if r.status_code != 200:
            raise RuntimeError(f'Failed to add to registry. {response_to_exception(r)}')

    def get_aas_as_dict(self, aas_id:str) -> dict:
        return self.get(self.url+aas_id)

    def get_aas(self, aas_id:str) -> object:
        return self.resolve_aas(self.get_aas_as_dict(aas_id))

    def find_object(self, id) -> object:
        r = self.get(self.url)
        for shell in r:
            aas = self.resolve_aas(shell)
            if aas.identification.id == id:
                 return aas
            if aas.asset.identification.id == id:
                return aas.asset
            for sm in aas.submodel:
                if sm.identification.id == id:
                    return sm

    def remove_aas(self, aas_id:str) -> bool:
        r = requests.delete(self.url+sanitize_id(aas_id), timeout = self._timeout)
        if r.status_code != 200:
            raise RuntimeError(f'REST DELETE failed. {response_to_exception(r)}')
        else:
            return True

    def remove_submodel(self, aas_id:str, submodel_id:str) -> bool:
        """
        Deletes a submodel from the Registry

        :aas_id: identification.id of AAS
        :submodel_id: identification.id of Submodel
        """
        aas_id = sanitize_id(aas_id)
        submodel_id = sanitize_id(submodel_id)
        r = requests.delete(self.url+aas_id+'/submodels/'+submodel_id, timeout = self._timeout)
        if r.status_code != 200:
            raise RuntimeError(f'REST DELETE failed. {response_to_exception(r)}')
        else:
            return True

    @classmethod
    def resolve_aas(cls, aas:dict) -> object:
        """Resolves AAS with a defined endpoint"""
        aas_endpoint = aas['endpoints'][0]
        # submodels_endpoints = [s['endpoints'][0] for s in aas['submodels']]
        aas = cls.resolve_endpoint(aas_endpoint)
        # submodels = {cls.resolve_endpoint(e) for e in submodels_endpoints}
        # aas.submodel = submodels
        return aas

    @classmethod
    def resolve_endpoint(cls, endpoint:dict) -> object:
        """Resolves the data given at a specific endpoint"""
        assert endpoint['type'] == 'http', 'Only http endpoints are accepted'
        json_data = cls.get(endpoint['address'])
        model_type = json_data['modelType']['name']
        if model_type == 'Submodel':
            return json.loads(json.dumps(json_data), cls=basyx_json.AASFromJsonDecoder)
        elif model_type == 'AssetAdministrationShell':
            return deserialize.deserialize_json_aas(json_data)
        else:
            raise RuntimeError(f'Unknown model type {model_type}')

    @classmethod
    def get(cls, url:str) -> dict:
        """Wrapper for GET with some Exceptions"""
        r = requests.get(url, timeout = 2) # hardcoded timeout
        if r.status_code != 200:
            if r.status_code == 404:
                raise KeyError(f'Failed to GET from registry. {response_to_exception(r)}')
            raise RuntimeError(f'Failed to GET from registry. {response_to_exception(r)}')
        return r.json()

    @classmethod
    def put(cls, url:str, data:str) -> dict:
        """Wrapper for PUT with some Exceptions"""
        r = requests.put(url,data=data, timeout = 2) # hardcoded timeout
        if r.status_code != 200:
            raise RuntimeError(f'Failed to PUT to registry. {response_to_exception(r)}')
        return r.json()

    # @classmethod
    # def delete(cls, url:str) -> dict:
    # """Wrapper for DEL with some Exceptions"""
    #     r = requests.delete(url, timeout = 2) # hardcoded timeout
    #     if r.status_code != 200:
    #         raise RuntimeError(f'REST DELETE failed. {response_to_exception(r)}')
    #     else:
    #         return r.json()
