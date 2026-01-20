import geometry_msgs.msg
import passform_msgs.msg
import std_msgs.msg
from basyx.aas import model
from passform_util.types.parameter import get_parameter


class Point(model.SubmodelElementCollectionUnordered):
    """basyx representation of passform_msgs.msg.AreaOfInterest
    """
    def __init__(
        self,
        msg : geometry_msgs.msg.Point,
        **kwargs
    ):
        super().__init__(
            id_short='point',   # needs some leading letters. Underscore is important for get_name()
            category='geometry_msgs',
            **kwargs
        )
        self.coordinates = ['x','y','z']
        self.set(msg)


    def set(self, msg: geometry_msgs.msg.Point) -> None:
        """Init the collection to the specified values
        """
        for coord in self.coordinates:
            self.value.add(
                model.Property(
                    id_short=coord,
                    value_type=model.datatypes.Double,
                    value=getattr(msg, coord)
                )
            )

    def to_msg(self) -> geometry_msgs.msg.Pose:
        """Create a ROS msg of the pose"""        
        msg = geometry_msgs.msg.Point()
        for coord in self.coordinates:
            setattr(msg, coord, get_parameter(self,coord))
        return msg

class Quaternion(model.SubmodelElementCollectionUnordered):
    """basyx representation of passform_msgs.msg.AreaOfInterest
    """
    def __init__(
        self,
        msg : geometry_msgs.msg.Quaternion,
        **kwargs
    ):
        super().__init__(
            id_short='quaternion',   # needs some leading letters. Underscore is important for get_name()
            category='geometry_msgs',
            **kwargs
        )
        self.coordinates = ['x','y','z','w']
        self.set(msg)


    def set(self, msg: geometry_msgs.msg.Quaternion) -> None:
        """Init the collection to the specified values
        """
        for coord in self.coordinates:
            self.value.add(
                model.Property(
                    id_short=coord,
                    value_type=model.datatypes.Double,
                    value=getattr(msg, coord)
                )
            )

    def to_msg(self) -> geometry_msgs.msg.Quaternion:
        """Create a ROS msg of the pose"""        
        msg = geometry_msgs.msg.Quaternion()
        for coord in self.coordinates:
            setattr(msg, coord, get_parameter(self,coord))
        return msg

class Pose(model.SubmodelElementCollectionUnordered):
    """basyx representation of passform_msgs.msg.AreaOfInterest
    """
    def __init__(
        self,
        msg : geometry_msgs.msg.Pose,
        **kwargs
    ):
        super().__init__(
            id_short='pose',   # needs some leading letters. Underscore is important for get_name()
            category='pose',
            **kwargs
        )
        self.set(msg)
        self._ros = msg


    def set(self, msg: geometry_msgs.msg.Pose) -> None:
        """Init the collection to the specified values
        """
        self.value.add(Point(msg.position))
        self.value.add(Quaternion(msg.orientation))

    def to_msg(self) -> geometry_msgs.msg.Pose:
        """Create a ROS msg of the pose"""        
        return self._ros

class Polygon(model.SubmodelElementCollectionUnordered):
    """basyx representation of geometry_msgs.msg.Polygon
    """ 
    def __init__(
        self,
        msg : geometry_msgs.msg.Polygon,
        **kwargs
    ):
        super().__init__(
            id_short='polygon',   # needs some leading letters. Underscore is important for get_name()
            **kwargs
        )

        for idx, point in enumerate(msg.points):
            point_element =  model.SubmodelElementCollectionUnordered(
                id_short='point_'+str(idx),
            )
            point_element.value.add( model.Property('x', point.x) )
            point_element.value.add( model.Property('y', point.y) )
            point_element.value.add( model.Property('z', point.z) )
            self.value.add(point_element)

    def to_msg(self) -> geometry_msgs.msg.Polygon:
        """Create ROS msg from basyx element"""
        msg = geometry_msgs.msg.Polygon()
        for point in self.value:
            if point.id_short.split('_')[0] == 'point':
                point_msg = geometry_msgs.msg.Point(
                    x = get_parameter(point, 'x'),
                    y = get_parameter(point, 'y'),
                    z = get_parameter(point, 'z'),
                )
                msg.points.add(point_msg)
        return msg

class AreaOfInterest(model.SubmodelElementCollectionUnordered):
    """basyx representation of passform_msgs.msg.AreaOfInterest
    """
    def __init__(
        self,
        msg : passform_msgs.msg.AreaOfInterest,
        **kwargs
    ):
        super().__init__(
            id_short='area_of_interest',   # needs some leading letters. Underscore is important for get_name()
            category='area_of_interest',
            **kwargs
        )
        self.set(msg)
        self.set_polygon(msg.polygon)

    def set(self, msg: passform_msgs.msg.AreaOfInterest) -> None:
        """Init the collection to the specified values
        """
        self.value.add(
            model.Property(
                id_short='label',
                value_type=model.datatypes.String,
                value=str(msg.label)
            )
        )

        self.value.add(
            model.Property(
                id_short='uid',
                value_type=model.datatypes.String,
                value=str(msg.uuid)
            )
        )
        
    def set_polygon(self, msg: geometry_msgs.msg.Polygon):
        """Setter for the collection polygon"""
        for val in self.value:
            if val.id_short == 'polygon':
                val = Polygon(msg)
                return
        self.value.add(Polygon(msg))

    def get_polygon(self) -> Polygon:
        """Getter for polygon"""
        return get_parameter(self, 'polygon')

    def get_label(self) -> str:
        """Getter for label"""
        for val in self.value:
            if val.id_short == 'label':
                return val.value
        return ''

    def get_uid(self) -> str:
        """Setter for label"""
        for val in self.value:
            if val.id_short == 'uid':
                return val.value
        return ''

    def to_msg(self) -> passform_msgs.msg.AreaOfInterest:
        """Create a ROS msg of the aoi"""        
        msg = passform_msgs.msg.AreaOfInterest(
            label = self.get_label(),
            uuid = self.get_uid(),
            polygon = self.get_polygon().to_msg()
        )
        return msg

class Location(model.SubmodelElementCollectionUnordered):
    """Collection to describe an item
    """
    def __init__(
        self,
        msg : passform_msgs.msg.Location,
        **kwargs
    ):
        super().__init__(
            id_short='location',
            category='location',
            **kwargs
        )
        self.set_header(msg.header)
        self.set_aoi(msg)
        self.set_pose(msg)

    def set_header(self, msg: std_msgs.msg.Header):
        """Setter for Property header"""
        self.value.add(model.Property(
            'frame_id',
            value_type=model.datatypes.String,
            value=msg.frame_id)
        )

    def get_frame_id(self) -> str:
        """Getter for frame_id"""
        for val in self.value:
            if val.id_short == 'frame_id':
                return val.value

    def get_aoi(self) -> model.SubmodelElementCollectionUnordered:
        """Getter for the collection area_of_interest"""
        for val in self.value:
            if val.id_short == 'area_of_interest':
                return val

    def set_aoi(self, msg: passform_msgs.msg.Location):
        """Setter for the collection area_of_interest"""
        for val in self.value:
            if val.id_short == 'area_of_interest':
                val = AreaOfInterest(msg.aoi)
                return
        self.value.add(AreaOfInterest(msg.aoi))

    def set_pose(self, msg: passform_msgs.msg.Location):
        """Setter for the colection pose"""
        for val in self.value:
            if val.id_short == 'pose':
                val = Pose(msg.pose)
                return
        self.value.add(Pose(msg.pose))

    def get_pose(self):
        """Getter for the colection pose"""
        for val in self.value:
            if val.id_short == 'pose':
                return val

    def to_msg(self) -> passform_msgs.msg.Location:
        """Create a ROS msg of the Location"""        
        msg = passform_msgs.msg.Location(
            header = std_msgs.msg.Header(
                frame_id = self.get_frame_id()
            ),
            pose = self.get_pose().to_msg(),
            aoi = self.get_aoi().to_msg(),
        )
        return msg
