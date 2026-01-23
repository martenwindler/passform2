#!/usr/bin/env python3
import json

from basyx.aas import model
import basyx.aas.adapter.json as basyx_json

def deserialize_json_aas(aas_json) -> model.AssetAdministrationShell:
    """
    Turns an AAS JSON to a BaSyx object

    Only uses locally stored data an will perform no dereferencing
    """
    # create asset and submodel objects for replacement later on
    asset_obj = json.loads(json.dumps(aas_json['asset']), cls=basyx_json.AASFromJsonDecoder)
    # create and encode a temporary shell for simplified asset reference data
    tmp_aas = model.AssetAdministrationShell(
        id_short='',
        identification=model.Identifier('', model.IdentifierType.CUSTOM),
        asset=model.AASReference.from_referable(model.Asset(
            kind=model.AssetKind.INSTANCE,
            identification=model.Identifier('', model.IdentifierType.IRI)
            )),
    )
    tmp_aas_json = json.loads(json.dumps(tmp_aas,cls=basyx_json.AASToJsonEncoder))
    aas_json['asset'] = tmp_aas_json['asset']
    # deserialize submodels
    try:
        submodels = list()
        for s in aas_json['submodels']:
            print(s)
            submodels.append(deserialize_json_submodel(s))
        # decode json data to AAS obect
        del aas_json['submodels']
    except:
        submodels = None
    aas_obj = json.loads(json.dumps(aas_json), cls=basyx_json.AASFromJsonDecoder)
    aas_obj.asset = asset_obj # replace asset object with original
    if ~isinstance(submodels, type(None)):
        aas_obj.submodel = submodels
    return aas_obj

def deserialize_json_submodel(s: dict) -> object:
    """
    Turns a JSON with potential submodel information to a BaSyx object

    Only uses locally stored data an will perform no dereferencing
    """
    try:
        parent = basyx_json.AASFromJsonDecoder._construct_reference(s['parent'])
    except KeyError as e:
        parent = None
    if s['modelType']['name'] in ['Submodel','SubmodelElementCollection']:
        obj = json.loads(json.dumps(s), cls=basyx_json.AASFromJsonDecoder)
        obj.parent = parent
        return obj
    elif s['modelType']['name'] == 'AssetAdministrationShell':
        obj.deserialize_json_aas(s)
        obj.parent = parent
        return obj
    else:
        raise KeyError(f'Unsupported submodel type: {s["modelType"]["name"]}')

def seperate_submodel_from_aas(aas: model.AssetAdministrationShell) -> tuple[model.AssetAdministrationShell, set[model.Submodel]]:
    submodels = aas.submodel
    aas.submodel = set()
    return aas, submodels

# def dereference_aas(aas_object: model.AssetAdministrationShell, obj_store: backend.backends.Backend):
#     """
#     Serializes a referencing asset administration in compliance with the AAS Server REST API
#     https://github.com/eclipse-basyx/basyx-java-sdk/issues/240#issuecomment-1463593068
#
#     :param aas_object: shell object of which all data will be gathered
#     :param obj_store: store in which the necessary assets and submodels are stored
#
#     :raises KeyError: if asset/submodel linked in shell is not found in store
#     """
#     aas = basyx_json.AASToJsonEncoder._asset_administration_shell_to_json(aas_object)
#     # asset and submodels with complete data as required by AAS Server
#     aas['asset'] = aas_object.asset.resolve(obj_store)
#     aas['submodels'] = [s.resolve(obj_store) for s in aas_object.submodel]
#     return json.dumps(aas,cls=basyx_json.AASToJsonEncoder)
