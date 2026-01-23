from typing import Optional
from rclpy.node import Node
from basyx.aas import model
from basyx.aas.model import base
from passform_util.basyx.rest_server import RestServer
from passform_util.log_level import LogLevel
from passform_msgs.srv import Discover

class RestServerConnector():
    """REST API helper to a local RestServer connected to a registry"""
    def __init__(
        self,
        parent:Node = None,
        timeout: int|float=2
    ):
        self._node = parent # TODO: can this be internalized?
        self._client : Optional[RestServer] = None
        self.managed_aas = dict()
        self._timeout = timeout

    def __del__(self):
        self.destroy()

    def connect_with_discover(self, resp:Discover.Response) -> None:
        """Connect using information received with a Passform Discover"""
        self.log(f'connection info {resp}',LogLevel.DEBUG)
        for ds in resp.server_description:
            if ds.name.lower() == 'basyx':
                self.log('BaSyx available.',LogLevel.DEBUG)
                data = {kv.key:kv.value for kv in ds.values}
                self.log(f'BaSyx data: {data}',LogLevel.DEBUG)
                if data['registry_type'] == 'REST':
                    host = data['registry_host']
                    self.connect(host)
                    return
        self.log('No BaSyx Information available.',LogLevel.WARN)

    def connect_with_parameter(self, data:dict) -> None:
        """ Connect with data from dict, provided by parameter response """
        registry = data['registry_host']
        self.connect(registry)

    def connect(self, registry_host:str):
        """Connect with a given registry host"""
        self.log(f'Creating BaSyx Server with Registry at {registry_host}',LogLevel.INFO)
        self._client = RestServer(registry=registry_host, timeout=self._timeout)
        self.log(f'Created BaSyx Server with Registry at {registry_host}',LogLevel.DEBUG)

    def add(
        self,
        obj: model.AssetAdministrationShell | model.Submodel,
        aas_id: str = None,
        submodel_id: str = None
    ) -> None:
        """
        Add object via REST.

        :obj: local model to be added
        :aas: identification.id of the AAS the submodel will be added to. Has to be already stored on the server.
        """
        if isinstance(obj, model.AssetAdministrationShell):
            self.add_aas(obj)
        elif isinstance(obj, model.Submodel):
            if aas_id != None:
                self.add_submodel(aas_id=aas_id, submodel=obj)
            else:
                raise KeyError(f'aas_id needs to be specified for adding a submodel.')
        elif isinstance(obj, model.SubmodelElement):
            self.add_element(obj, aas_id, submodel_id)
        else:
            raise KeyError(f'Can only add AAS or Submodel. Got {type(obj)}.')

    def add_aas(self, obj: model.AssetAdministrationShell):
        self._client.add_aas(obj)
        self.managed_aas[obj.identification.id] = model.AssetAdministrationShell

    def add_submodel(self, submodel:model.Submodel, aas_id:str) -> None:
        """
        Add submodel to AAS via REST PUT

        :submodel: local submodel to be added
        :aas_id: identification.id of the AAS the submodel will be added to. Has to be already stored on the server.
        """
        for obj in self._client.find_problematic_elements(submodel):
            self.log(f'{aas_id} contains a nested submodel. Skipped {submodel.id_short}:{obj.id_short}.',LogLevel.WARN)
        self._client.add_submodel(aas_id,submodel)
        self.log(f'Added submodel "{submodel.id_short}" to aas "{aas_id}".',LogLevel.DEBUG)

    def add_element(self, element: model.SubmodelElement, aas_id:str, submodel_id:str) -> None:
        """
        Add submodelElement to AAS via REST PUT

        :param element: submodelElement to be added.
        :param aas_id: identification.id of the AAS the submodel will be added to. Has to be already stored on the server.
        :param submodel: local submodel to be added
        """
        self._client.add_element(
            aas_id=aas_id,
            submodel_id=submodel_id,
            obj=element
        )
        self.log(f'Added element "{element.id_short}" to submodel "{aas_id}/{submodel_id}".',LogLevel.DEBUG)

    def discard(self, aas_id:str, submodel_idshort:str=None) -> bool:
        """
        Remove an AAS or submodel from the server

        :aas_id: identification.id of the AAS
        :submodel_idshort: id_short of the submodel
        """
        if submodel_idshort != None:
            self._client.remove_submodel(aas_id, submodel_idshort)
        else:
            self._client.remove_aas(aas_id)
        return True

    def destroy(self):
        """Remove all items added via REST"""
        for id, model_type in self.managed_aas.items():
            try:
                self.discard(id)
            except Exception as e:
                self.log(f'Failed to remove AAS {id}. {e}',LogLevel.WARN)

    def log(self, msg, lvl:LogLevel=LogLevel.INFO):
        if self._node is not None:
            if lvl == LogLevel.DEBUG:
                self._node.get_logger().debug(msg)
            elif lvl == LogLevel.INFO:
                self._node.get_logger().info(msg)
            elif lvl == LogLevel.WARN:
                self._node.get_logger().warn(msg)
            elif lvl == LogLevel.ERROR:
                self._node.get_logger().error(msg)
            else:
                self._node.get_logger().fatal(msg)
        else:
            print(f'{lvl}: {msg}')

    def get_aas(self, aas_id):
        return self._client.get_aas(aas_id)

    def get_submodel(self, aas_id, submodel_idshort):
        return self._client.get_submodel(aas_id, submodel_idshort)

    def set_value(self,
        aas_id: str,
        submodel_idshort: str,
        value_idshort: str,
        value: base.ValueDataType
    ) ->  None:
        return self._client.set_value(aas_id, submodel_idshort, value_idshort, value)

def ConnectorFactory(
    config : object,
    node : Node=None,
    timeout: int|float = 3
)-> RestServerConnector:
    connector = RestServerConnector(node, timeout)
    if isinstance(config, Discover.Response):
        connector.connect_with_discover(config)
    elif isinstance(config, dict):
        connector.connect_with_parameter(config)
    else:
        connector.connect(config)
    return connector
