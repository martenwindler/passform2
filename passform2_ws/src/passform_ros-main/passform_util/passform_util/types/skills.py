import uuid
from enum import Enum
from pathlib import Path
from typing import Optional

import passform_msgs.msg
import yaml
from basyx.aas import model
from passform_util.basyx import sanitize_basyx_string
from passform_util.basyx.conversion import (dict_to_dataelement,
                                            dict_to_variable)
from passform_util.types.parameter import get_parameter


class SkillType(Enum):
    STOP = 0
    MOVE = 1
    MOVE_TCP = 2
    MOVE_BODY = 3

    GRASP = 10
    # ITEM_MOVE = 11
    PRESSURE = 12
    POSITION = 13
    RELEASE = 14

    TOOL_GET = 20
    TOOL_USE = 21
    TOOL_PUT = 22
    STORE = 23
    PROVIDE = 24

    ASSEMBLY_PICKPLACE = 30
    ASSEMBLY_SCREW = 31
    ASSEMBLY_MOUNT = 32
    ASSEMBLY_PLUG = 33

    def __int__(self):
        return self.value

    def __str__(self):
        return self.name

class Operation(model.Operation):
    """basyx Operation for potential skill operations
    """

    def __init__(self,
        skill_type: SkillType,
        name: str,
        input_required : dict,
        input_optional : dict = None,
        output: dict = None
    ):            
        super().__init__(
            # append mode_name with underscore
            id_short = sanitize_basyx_string((' '.join([skill_type.name, name]).strip())),
            # create static values
            input_variable=[dict_to_variable(inp, category='optional') for inp in input_optional],
            output_variable=[dict_to_variable(out) for out in output]
        )
        for inp in input_required:
            self.input_variable.append(dict_to_variable(inp, category='required'))

        # empty variable lists are necessary. do not remove

    def is_ros(self) -> bool:
        """Checks if all variables are based on ROS data
        """
        for var in self.get_inputs()+self.get_outputs():
            try:
               if list(var.value.qualifier)[0].type != 'ROS':
                    return False
            except IndexError:
                return False
        return True

    def can_perform(self, task: passform_msgs.msg.Task) -> bool:
        """Checks if all required parameters are provided
        """
        # check whether all required task parameters can be performed
        available_inputs = self.get_input_names()
        for parameter in task.required_parameter:
            if parameter.name not in available_inputs:
                return False
        # check whether all required operation parameter are provided
        provided_parameter = [param.name for param in task.required_parameter+task.optional_parameter]
        for req_input in self.get_required_input_names():
            if req_input not in provided_parameter:
                return False
        return True

    def get_input_names(self) -> list[str]:
        """Getter for all available input names"""
        return [inp.value.id_short for inp in self.get_inputs()]

    def get_inputs(self) -> list[model.OperationVariable]:
        """Getter for all available input variables"""
        return self.get_required_inputs() + self.get_optional_inputs()

    def get_required_input_names(self) -> list[str]:
        """Getter for required input names"""
        return [inp.value.id_short for inp in self.get_required_inputs()]

    def get_required_inputs(self) -> list[model.OperationVariable]:
        """Getter for required input variables"""
        var_list = []
        for inp in self.input_variable:
            if inp.value.category == 'required':
                var_list.append(inp)
        return var_list

    def get_optional_input_names(self) -> list[str]:
        """Getter for optional input names"""
        return [inp.value.id_short for inp in self.get_optional_inputs()]

    def get_optional_inputs(self) -> list[model.OperationVariable]:
        """Getter for optional input variables"""
        var_list = []
        for inp in self.input_variable:
            if inp.value.category != 'required':
                var_list.append(inp)
        return var_list

    def get_outputs(self) -> list[model.OperationVariable]:
        """Getter for output variables"""
        return self.output_variable if self.output_variable is not None else []

    def get_variables(self) -> list[model.OperationVariable]:
        """Combines all variables of an operation into one list"""
        inp = self.input_variable if self.input_variable is not None else []
        inp_out = self.in_output_variable if self.in_output_variable is not None else []
        out = self.output_variable if self.output_variable is not None else []
        return inp+inp_out+out

class Skill(model.Submodel):
    """basyx Submodel for skill representations.

    :param type: name of the skill (has to be in SkillType)
    :param data: dictionary with all relevant skill information
    """
    def __init__(self,
            type: str,
            data: Optional[dict] = None,
            **kwargs):
        type = SkillType[type.upper()]  #TODO: cast here or externally?
        super().__init__(
            identification=model.Identifier(str(uuid.uuid4()), model.IdentifierType.CUSTOM),
            id_short=sanitize_basyx_string(data['id_short']) if 'id_short' in data else sanitize_basyx_string(type.name),
            category='Skill',
            **kwargs)
        # set basic data (TODO: needed, since operation info avaiable?)
        prop = model.Property(
                id_short = 'skilltype',  # Identifying string of the element within the submodel namespace
                value_type = model.datatypes.Int,  # Data type of the value
                value = type.value,  # Value of the property
            )
        self.submodel_element.add(prop)

        # set description if available
        self.description = data['description'] if 'description' in data else dict()
        # set parameter
        self.create_properties(data)
        self.create_dependencies(data)
        self.set_driver(data)
        self.create_operation(type, data)
        # self.submodel_element.add( create_operation(type, data) )

    def __str__(self):
        return f'{self.id_short} ({self.identification.id})'

    @classmethod
    def from_file(cls, param_file:str|Path):
        """Create a Skill object from a yaml file"""
        with open(param_file, 'r') as file:
            skill_data = yaml.safe_load(file)
        skill_type = skill_data.pop('type') # remove field type from dict
        return cls(
            type = skill_type,
            data = skill_data,
        )

    def create_properties(self, data:dict) -> None:
        """
        Create and add a submodel with all skill properties.
        """
        if 'properties' not in data:
            return
        properties = data['properties']
        collection = model.SubmodelElementCollectionUnordered(
            id_short='SkillProperties',
            description={
                'en-us':'List of all skill properties.',
                'de':'Sammlung aller Eigenschaften des Skills.'
            }
        )
        # create propertie elements and add collection to submodel
        for p in properties:
            collection.value.add(dict_to_dataelement(p))
        self.submodel_element.add(collection)

    def create_dependencies(self, data:dict) -> None:
        """
        Create and add a submodel with all required primitives.
        """
        if 'primitives' not in data:
            return
        properties = data['primitives']
        collection = model.SubmodelElementCollectionOrdered(
            id_short='primitives',
            description={
                'en-us':'List of all required primitives.',
                'de':'Sammlung aller notwendigen Primitive.'
            }
        )
        # create propertie elements and add collection to submodel
        for prop in properties:
            collection.value.add(
                model.Property(
                    value_type = model.datatypes.Int,
                    id_short = prop['type'],
                    value = SkillType[prop['type'].upper()].value # prevents unknown skills
                )
            )
        self.submodel_element.add(collection)

    def set_driver(self, data:dict) -> None:
        """
        Create and add a submodel with driver information.
        """
        if 'driver_topic' not in data:
            return
        self.submodel_element.add(
            model.Property(
                id_short = 'driver',  # Identifying string of the element within the submodel namespace
                value_type = model.datatypes.String,  # Data type of the value
                value = data['driver_topic'],  # Value of the property
                description = {
                    'en-us':'ROS2 topic for the driver of this skill.',
                    'de':'ROS2 topic für die Ansteuerung der Fähigkeit.'}
            )
        )

    def create_operation(self, skill_type:SkillType, data:dict) -> None:
        """ Create and add a collection with all available operation modes of this skill.
        """
        # collection as storage
        collection = model.SubmodelElementCollectionUnordered(
            id_short='operations',
            description={
                'en-us':'List of all skill operation modes.',
                'de':'Sammlung aller Ausführungsoptionen der Fähigkeit.'
            }
        )
        # create input and output variable lists
        if 'input' in data:
            req_inputs = data['input']['required'] if 'required' in data['input'] else data['input']
            opt_inputs = data['input']['optional'] if 'optional' in data['input'] else list()
        else:
            req_inputs = list()
            opt_inputs = list()
        # if no required inputs are provided, the above code fails and puts optional as required
        if 'optional' in req_inputs:            
            opt_inputs+=req_inputs['optional']
            req_inputs = {'':[]}
        outputs = data['output'] if 'output' in data else list()

        # create dict of input modes (might be a list if only one mode exists)
        if isinstance(req_inputs, list):
            req_inputs = {'':req_inputs}    # mode_name is empty string, if only one mode

        # create an Option for each mode in req_inputs, a dict with {mode_name:inputs}
        for mode_name, inputs_required in req_inputs.items():
            collection.value.add(Operation(
                skill_type = skill_type,
                name = mode_name,
                input_required = inputs_required,
                input_optional = opt_inputs,
                output = outputs
            ))
        self.submodel_element.add(collection)

    def get_operations(self) -> list[Operation]:
        """Getter for Collection "operations"""
        return list(get_parameter(self,'operations').value)

    def is_composite(self) -> bool:
        """True if skill uses primitives"""
        return self.get_primitives() != []

    def get_skilltype(self) -> SkillType:
        """Getter for Property "skilltype"""
        return SkillType(get_parameter(self, 'skilltype').value)

    def to_msg(self) -> passform_msgs.msg.Skill:
        """Convert to ROS msg"""
        msg = passform_msgs.msg.Skill(
            type = passform_msgs.msg.SkillType(
                skill_type=int(self.get_skilltype())
            ),
            uuid = self.identification.id,
        )
        return msg

    def get_primitives(self) -> list[SkillType]:
        """Getter for Collection "primitives" as list of SkillType
        Returns the skilltype of all elements stored in the 'primitives' element
        """
        prim_properties = get_parameter(self, 'primitives')
        primitives = list()
        if isinstance(prim_properties, model.SubmodelElementCollectionOrdered):
            for p in prim_properties.value:
                skilltype = SkillType(p.value)
                primitives.append(skilltype)
        return primitives

    def get_driver(self) -> str:
        """Getter for Property "driver" """
        return get_parameter(self,'driver').value

if __name__ == '__main__':#
    import yaml
    with open('/home/jasper/git/passform_ros/passform_util/passform_util/types/skills_material.yaml', 'r') as file:
        skill_data = yaml.safe_load(file)
    skills = list()
    for skill in skill_data:
        try:
            skill_type = skill.pop('type') # remove field type from dict
            s = Skill(
                type = skill_type,
                data = skill,
            )
            skills.append(s)
        except Exception as e:
            print(f'Failed to create skill: {e}')

    import json

    import basyx.aas.adapter.json as basyx_json
    from passform_util.basyx import deserialize
    for s in skills:
        print(f'Skill: {s.id_short}')
        for o in get_parameter(s,'operations'):
            print(o.is_ros())
                #     if list(v.value.qualifier)[0].type != 'ROS':
                #         raise 'wrong type'
                # except Exception:
                #     raise KeyError(f'operation "{o.id_short}" variable "{v.value.id_short}" not of ROS type')
        # # to server
        # data = json.loads(json.dumps(s, cls=basyx_json.AASToJsonEncoder))
        # print(data)
        # # from server
        # deserialize.deserialize_json_submodel(data)
