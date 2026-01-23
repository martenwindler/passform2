import mimetypes
import os
import re

from basyx.aas import model
from basyx.aas.model import base
from passform_util.basyx import sanitize_basyx_string


def data_to_blob(data_path, id_short:str=None) -> model.Blob:
    """Turn a file at data_path to a basyx blob
    """
    if id_short is None:
        id_short = sanitize_basyx_string(os.path.splitext(os.path.basename(data_path))[0])
    obj = model.Blob(
        id_short=id_short,
        mime_type=mimetypes.guess_type(data_path)[0]
    )
    with open(data_path, "rb") as file:
        f = file.read()
        obj.value = bytearray(f)[0:-1] # it seems the last element breaks decoding
    return obj

def dict_to_dataelement(data:dict, **kwargs) -> model.DataElement:
    """
    Create a DataElement object from dict. Also tries to convert to ROS-based element.

    type: <Property | SubmodelElementCollection* | MultiLanguageProperty | Range>
    id_short: <str>
    value_type: <model.datatypes>
    data:
        value: <Any>
        description: <dict(lang-key:<str>)>

    For ROS:
    type: ROS-datatype
    id_short: Optional
    """
    id_short = data['id_short'] if 'id_short' in data else None
    val_type = getattr(model.datatypes, data['value_type']) if 'value_type' in data else None
    if 'data' in data:
        kwargs.update(data['data'])
    if data['type'] == 'Property':
        obj = model.Property(id_short, val_type, **kwargs)
    elif data['type'] == 'MultiLanguageProperty':
        obj = model.MultiLanguageProperty(id_short, **kwargs)
    elif data['type'] == 'Range':
        obj = model.Range(id_short, val_type, **kwargs)
    elif data['type'] in ['SubmodelElementCollection', 'SubmodelElementCollectionUnordered']:
        obj = model.SubmodelElementCollectionUnordered(id_short, **kwargs)
        for d in data['data']:
            obj.value.add(dict_to_dataelement(d, **kwargs))
    elif '/' in data['type']:
        obj = ros_to_dataelement(data, **kwargs)
    else:
        try:
            # convert to ROS as last restort. raises for all unknown datatypes
            obj = ros_to_dataelement(data, **kwargs)
        except KeyError:
            raise KeyError(f'DataElement "{data["type"]}" unkown.')
    return obj

def dict_to_variable(data:dict, category=None) -> model.OperationVariable:
    """
    creates an OperationVariable object from dict

    type: <Property | MultiLanguageProperty | Range>
    id_short: <str>
    value_type: <model.datatypes>
    data:
        value: <Any>
        description: <dict(lang-key:<str>)>
    """
    return model.OperationVariable(
        value = dict_to_dataelement(data, category=category, kind=base.ModelingKind.TEMPLATE)
    )

datatype_conversion = {
# convert ROS type to basyx DataType
    'int8': 'Short',
    'int32': 'Integer',
    'uint8': 'UnsignedShort',
    'uint32': 'UnsignedInt',
    'string': 'String',
    'double': 'Double',
    'boolean': 'Boolean',
    'octet': 'Byte', # byte in msg definitions
    'float': 'Float'
}

def ros_to_dataelement(data:dict, kind=base.ModelingKind.INSTANCE, **kwargs) -> model.DataElement:
    """
    Create a DataElement object from ROS dict.
    NOT TESTED FOR ALL ROS DATATYPES.

    :dict{}:
        type: <ROS-datatype>
        id_short: Optional<str>
    """
    is_array = False
    data_type = data['type']
    if 'data' in data:
        kwargs.update(data['data'])
    id_short = data['id_short'] if 'id_short' in data else kwargs['id_short'] if 'id_short' in kwargs else None
    if 'sequence' in data_type:
        # TODO: Store arrays in DataElement
        data_type = data_type.replace('sequence<','').replace('>','')
        is_array = True
    if '/' in data_type:
        # complex ROS message like 'geometry_msgs/Pose'
        pkg = data_type.split('/')[0]
        msg = data_type.split('/')[1]
        id_short=id_short if id_short != None else msg
        if is_array: id_short+='_ARRAY'
        obj = model.SubmodelElementCollectionUnordered(id_short, kind=kind, **kwargs)
        ros_msg = getattr(__import__(pkg).msg, msg)
        for id_short, child_type in ros_msg.get_fields_and_field_types().items():
            obj.value.add(ros_to_dataelement(
                {'type': child_type, 'id_short': id_short},
                kind=kind,
                **kwargs  # nest additional information through all layers
            ))
    else:
        # simple variable
        if is_array: id_short+='_ARRAY'
        obj = model.Property(id_short, getattr(model.datatypes, datatype_conversion[data_type]), **kwargs)
        obj._kind = kind # dirty way of correcting the kind
        # for kw in kwargs:
        # # nest additional information through all layers
        #     setattr(obj, kw, kwargs[kw])
    obj = add_ros_qualifier(obj, data_type)
    return obj

def add_ros_qualifier(obj, data_type):
    obj.qualifier = {base.Qualifier(
        type_= 'ROS',
        value_type= model.datatypes.String,
        value= data_type,
        )}
    return obj
