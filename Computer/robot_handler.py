from map import obstacles

class Robot(object):
    def __init__(self, robot_id: int, position: list[float], orientation: float, radius: float = 100) -> None:
        super().__init__()
        self._id = robot_id
        self._position = position
        self._orientation = orientation
        self._radius = radius
        self._shape = obstacles.Circle(robot_id, position, orientation, radius)
        self._target = None
        return None

    def get_id(self):
        return self._id
        
    def get_coordinates(self):
        return self._position, self._orientation

    def get_radius(self):
        return self._radius
    
    def get_shape(self):
        return self._shape

    def set_target(self, target: list[float]) -> None:
        if len(target) == 2:
            self._target = (target, None)
        elif len(target) == 3:
            self._target = (target[0:2], target[2])
        else:
            raise Exception('Wrong number of arguments for target')
        return None




