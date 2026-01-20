#!/usr/bin/env python3

import unittest
from unittest.mock import patch

import json
from basyx.aas import model
import basyx.aas.adapter.json

import passform_util.network
from passform_util.basyx.rest_server import RestServer
from passform_util.basyx.rest_registry import RestRegistry

REGISTRY_IP = passform_util.network.get_ip()
AAS_SERVER_IP = passform_util.network.get_ip()

class TestRegistry(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.maxDiff = None
        cls._host = REGISTRY_IP
        cls._client = RestRegistry(cls._host)
        # models
        asset = model.Asset(
            id_short='Asset',
            kind=model.AssetKind.INSTANCE,
            identification=model.Identifier('REGISTRYTEST_AssetWoSubmodel', model.IdentifierType.IRI)
        )
        cls.aas_empty = model.AssetAdministrationShell(
            id_short='ExampleAAS',
            identification=model.Identifier('REGISTRYTEST_AAS_empty', model.IdentifierType.CUSTOM),
            asset=asset,
            # submodel={submodel, submodel2}
        )
        submodel = model.Submodel(
            id_short='Submodel_IDShort',
            identification=model.Identifier('REGISTRYTEST_ident-model2', model.IdentifierType.CUSTOM),
            description={'de':'Beschreibung'}
        )
        asset2 = model.Asset(
            id_short='AssetWith',
            kind=model.AssetKind.INSTANCE,
            identification=model.Identifier('REGISTRYTEST_AssetWithSubmodel', model.IdentifierType.IRI)
        )
        cls.aas = model.AssetAdministrationShell(
            id_short='ExampleAAS',
            identification=model.Identifier('REGISTRYTEST_AAS', model.IdentifierType.CUSTOM),
            asset=asset2,
            submodel={submodel}
        )

    @patch('builtins.print')
    def test_bad_port(self, mock_print):
        port = 65535
        bad_client = RestRegistry(host=self._host, port=port)
        mock_print.assert_called_with(f'Warning: Registry port {self._host}:{port} not active.')

    def test_add_empty_aas(self):
        test_aas = self.aas_empty
        aas_orig = json.loads(json.dumps(test_aas, cls=basyx.aas.adapter.json.AASToJsonEncoder))
        endpoint = {
            'type': 'http',
            'address': 'test'
        }
        self._client.add_aas(test_aas, endpoint)
        aas_id = test_aas.identification.id
        aas = self._client.get_aas_as_dict(aas_id)
        assert aas['modelType']['name'] == 'AssetAdministrationShellDescriptor', 'Wrong type.'
        del aas['endpoints'], aas['modelType'], aas_orig['modelType'], aas['submodels']
        self.assertDictEqual(aas, aas_orig)
        self._client.remove_aas(aas_id)

    def test_add_aas(self):
        test_aas = self.aas
        aas_orig = json.loads(json.dumps(test_aas, cls=basyx.aas.adapter.json.AASToJsonEncoder))
        endpoint = {
            'type': 'http',
            'address': 'test'
        }
        self._client.add_aas(test_aas, endpoint)
        aas_id = test_aas.identification.id
        aas = self._client.get_aas_as_dict(aas_id)
        assert aas['modelType']['name'] == 'AssetAdministrationShellDescriptor', 'Wrong type.'
        del aas['endpoints'], aas['modelType'], aas_orig['modelType']
        del aas['submodels'], aas_orig['submodels']
        self.assertDictEqual(aas, aas_orig)
        self._client.remove_aas(aas_id)


class TestServer(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._host = AAS_SERVER_IP
        cls._client = RestServer(registry=REGISTRY_IP)
        # models
        asset = model.Asset(
            id_short='Asset',
            kind=model.AssetKind.INSTANCE,
            identification=model.Identifier('SERVERTEST_AssetWoSubmodel', model.IdentifierType.CUSTOM)
        )
        cls.aas_empty = model.AssetAdministrationShell(
            id_short='ExampleAAS',
            identification=model.Identifier('SERVERTEST_AAS_empty', model.IdentifierType.CUSTOM),
            asset=asset,
            # submodel={submodel, submodel2}
        )
        submodel = model.Submodel(
            id_short='Submodel_IDShort',
            identification=model.Identifier('SERVERTEST_ident-model2', model.IdentifierType.CUSTOM),
            description={'de':'Beschreibung'}
        )
        asset2 = model.Asset(
            id_short='AssetWith',
            kind=model.AssetKind.INSTANCE,
            identification=model.Identifier('SERVERTEST_AssetWithSubmodel', model.IdentifierType.CUSTOM)
        )
        cls.aas = model.AssetAdministrationShell(
            id_short='ExampleAAS',
            identification=model.Identifier('SERVERTEST_AAS', model.IdentifierType.CUSTOM),
            asset=asset2,
            submodel={submodel}
        )

    @patch('builtins.print')
    def test_bad_port(self, mock_print):
        port = 65535
        bad_client = RestServer(port=port)
        mock_print.assert_called_with(f'Warning: Server port {passform_util.network.get_ip()}:{port} not active.')

    def test_add_aas_content(self):
        for test_aas in [self.aas_empty, self.aas]:
            aas_id = test_aas.identification.id
            test_aas_dict = json.loads(json.dumps(test_aas, cls=basyx.aas.adapter.json.AASToJsonEncoder))
            self._client.add_aas(test_aas)
            tmp_aas = self._client.get_aas(aas_id)
            tmp_aas_dict = json.loads(json.dumps(tmp_aas, cls=basyx.aas.adapter.json.AASToJsonEncoder))
            self.assertDictEqual(tmp_aas_dict, test_aas_dict)
            self._client.remove_aas(aas_id)

    def test_get_aas_type(self):
        for test_aas in [self.aas_empty, self.aas]:
            aas_id = test_aas.identification.id
            self._client.add_aas(test_aas)
            tmp_aas = self._client.get_aas(aas_id)
            self.assertIsInstance(tmp_aas, model.AssetAdministrationShell)
            self._client.remove_aas(aas_id)

    # def test_remove_aas(self):
    #     test_aas = self.aas
    #     aas_id = test_aas.identification.id
    #     self._client.add_aas(test_aas)
    #     self.assertTrue( self._client.remove_aas(aas_id) )
    #     with self.assertRaises(KeyError):
    #         self._client.registry.get_aas(aas_id)

    ## SUBMODE OPERATIONS CURRENTLY NOT SUPPORTED
    # def test_get_submodel_type(self):
    #     test_aas = self.aas
    #     aas_id = test_aas.identification.id
    #     for s in test_aas.submodel:
    #         submodel_idshort = s.id_short
    #         self._client.add_aas(test_aas)
    #         tmp_model = self._client.get_submodel(aas_id, submodel_idshort)
    #         self.assertIsInstance(tmp_model, model.Submodel)
    #     self._client.remove_aas(aas_id)
    #
    # def test_get_submodel_content(self):
    #     test_aas = self.aas
    #     aas_id = test_aas.identification.id
    #     for s in test_aas.submodel:
    #         submodel_idshort = s.id_short
    #         model_dict = json.loads(json.dumps(s, cls=basyx.aas.adapter.json.AASToJsonEncoder))
    #         self._client.add_aas(test_aas)
    #         tmp_model = self._client.get_submodel(aas_id, submodel_idshort)
    #         tmp_model_dict = json.loads(json.dumps(tmp_model, cls=basyx.aas.adapter.json.AASToJsonEncoder))
    #         self.assertDictEqual(tmp_model_dict, model_dict)
    #     self._client.remove_aas(aas_id)

    def test_resolve_registry_aas_type(self):
        for test_aas in [self.aas_empty, self.aas]:
            aas_id = test_aas.identification.id
            self._client.add_aas(test_aas)
            tmp_aas = self._client.registry.get_aas(aas_id)
            self.assertIsInstance(tmp_aas, model.AssetAdministrationShell)
            self._client.remove_aas(aas_id)

    def test_resolve_registry_aas_content(self):
        for test_aas in [self.aas_empty, self.aas]:
            aas_id = test_aas.identification.id
            test_aas_dict = json.loads(json.dumps(test_aas, cls=basyx.aas.adapter.json.AASToJsonEncoder))
            self._client.add_aas(test_aas)
            tmp_aas = self._client.registry.get_aas(aas_id)
            tmp_aas_dict = json.loads(json.dumps(tmp_aas, cls=basyx.aas.adapter.json.AASToJsonEncoder))
            self.assertDictEqual(tmp_aas_dict, test_aas_dict)
            self._client.remove_aas(aas_id)

    def test_search_submodel(self):
        self._client.add_aas(self.aas)
        self._client.add_aas(self.aas_empty)
        for s in self.aas.submodel:
            orig_dict = json.loads(json.dumps(s, cls=basyx.aas.adapter.json.AASToJsonEncoder))
        tmp = self._client.registry.find_object('SERVERTEST_ident-model2')
        tmp_dict = json.loads(json.dumps(tmp, cls=basyx.aas.adapter.json.AASToJsonEncoder))
        self.assertDictEqual(tmp_dict, orig_dict)
        self._client.remove_aas(self.aas.identification.id)
        self._client.remove_aas(self.aas_empty.identification.id)

    def test_search_aas(self):
        self._client.add_aas(self.aas)
        self._client.add_aas(self.aas_empty)
        orig_dict = json.loads(json.dumps(self.aas, cls=basyx.aas.adapter.json.AASToJsonEncoder))
        tmp = self._client.registry.find_object('SERVERTEST_AAS')
        tmp_dict = json.loads(json.dumps(tmp, cls=basyx.aas.adapter.json.AASToJsonEncoder))
        self.assertDictEqual(tmp_dict, orig_dict)
        self._client.remove_aas(self.aas.identification.id)
        self._client.remove_aas(self.aas_empty.identification.id)

    # def test_nested_submodels(self):
    #     """Put an AAS with a nested submodel to REST. Currently fails."""
    #     test_aas = self.aas_empty
    #     p1 = model.Property(
    #         id_short='ExampleProperty',
    #         value_type=model.datatypes.String,
    #         value='exampleValue',
    #     )
    #     p2 = model.Range(
    #         id_short='ExampleRange',
    #         value_type=model.datatypes.Int,
    #         min=1,
    #         max=5,
    #     )
    #     layered_submodel = model.Submodel(
    #         id_short='sm_layer1',
    #         identification=model.Identifier('LAYERTEST_ident-model1', model.IdentifierType.CUSTOM),
    #         submodel_element={
    #             model.Submodel(
    #                 id_short='sm_layer2',
    #                 identification=model.Identifier('LAYERTEST_ident-model2', model.IdentifierType.CUSTOM),
    #                 # submodel_element={p2}
    #                 ),
    #                 p1
    #         }
    #     )
    #     test_aas.submodel = layered_submodel
    #     test_aas_dict = json.loads(json.dumps(test_aas, cls=basyx.aas.adapter.json.AASToJsonEncoder))
    #     print(test_aas_dict)
    #     self._client.add_aas(test_aas)
    #     aas_id = test_aas.identification.id
    #     tmp_aas = self._client.registry.get_aas(aas_id)
    #     for sm in tmp_aas.submodel:
    #         for se in sm.submodel_element:
    #             print(se)
    #     tmp_aas_dict = json.loads(json.dumps(tmp_aas, cls=basyx.aas.adapter.json.AASToJsonEncoder))
    #     print(tmp_aas_dict)
    #
    #     self.assertDictEqual(test_aas_dict, tmp_aas_dict)
    #     self._client.remove_aas(test_aas.identification.id)

if __name__ == '__main__':
    unittest.main()
