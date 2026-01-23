import fnmatch

from basyx.aas import model


def get_parameter(obj:model.Submodel|model.SubmodelElementCollection, id_short:str) -> model.DataElement:
    """
    Returns the first occurence of id_short in submodel_elements and their objects.
    Allows wildcards like * (0..n characters) and ? (exact 1 character)
    """
    if isinstance(obj, model.Submodel):
        element_list = obj.submodel_element
    elif isinstance(obj, (model.SubmodelElementCollection, model.SubmodelElementCollectionUnordered, model.SubmodelElementCollectionOrdered)):
        element_list = obj.value
    else:
        raise ValueError('obj must be of type Submodel or SubmodelElementCollection')
    for se in element_list:
        if fnmatch.fnmatch(se.id_short, id_short) > 0:
            return se
        try:
            for o in se:
                if fnmatch.fnmatch(o.id_short, id_short) > 0:
                    return o
        except:
            pass
    return None
