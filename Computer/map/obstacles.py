import enum
import numpy as np
import math

class Obstacle(object):
    def __init__(self, obstacle_id: int, center: list[float], rotation: float)-> None:
        super().__init__()
        self._id = obstacle_id
        self._center = np.asarray(center)
        self._rotation = rotation
        return None

    def get_center(self) -> np.ndarray:
        return self._center

class Polygon(Obstacle):
    def __init__(self, obstacle_id: int, center: list[float], rotation: float, points: np.ndarray) -> None:
        super().__init__(obstacle_id, center, rotation)
        self._vertices = Polygon.rotate(points, rotation)
        self._vertices += np.asarray([center]).transpose()
        return None

    def get_vertices(self) -> np.ndarray:
        return self._vertices

    def rotate(vectors: np.ndarray, angle: float)-> np.ndarray:
        '''Rotates an array of column vectors by an angle in radians'''
        c, s = np.cos(angle), np.sin(angle)
        R = np.array(((c, -s), (s, c)))
        rotated_vectors = R.dot(vectors)
        return rotated_vectors

class Triangle(Polygon):
    def __init__(self, obstacle_id: int, center: list[float], rotation: float, points: list[list[int]])-> None:
        points_np = np.asarray(points).transpose()
        super().__init__(obstacle_id, center, rotation, points_np)
        return None
                                                                               
class Rectangle(Polygon):
    def __init__(self, obstacle_id: int, center: list[float], rotation: float, lengths: list[float])-> None:
        self._lengths = np.asarray(lengths)
        temp_0 = np.array([[.5, .5]]) * lengths
        temp_0 = temp_0.transpose()
        temp_90 = Polygon.rotate(temp_0,2*math.atan(temp_0[0]/temp_0[1]))# np.pi/2)
        temp_180 = Polygon.rotate(temp_0,np.pi)
        temp_270 = Polygon.rotate(temp_0,np.pi+2*math.atan(temp_0[0]/temp_0[1]))
        points_np = np.concatenate((temp_0, temp_90, temp_180, temp_270), axis=1)
        super().__init__(obstacle_id, center, rotation, points_np)
        return None

class Circle(Obstacle):
    def __init__(self, obstacle_id: int, center: list[float], rotation: float, radius: float)-> None:
        super().__init__(obstacle_id, center, rotation)
        self._radius = radius
        return None
    def get_radius(self) -> float:
        return self._radius

def load_obstacles(obstacles: list, figures: list)-> dict[int:Obstacle]:
    '''Initialize obstacle objects from given obstacle and figures jsons'''
    dict_of_obstacles = {}
    for obstacle in obstacles:
        obstacle_id = obstacle['id']
        center      = list(obstacle['center'].values())
        rotation    = np.radians(obstacle['rotation'])
        for figure in figures:
            if figure['id'] == obstacle_id:
                obstacle_figure = _FigureType(figure['type'])
                if obstacle_figure is _FigureType.TRIANGLE:
                    dict_points = [v for k, v in figure.items() if k.startswith('point')]    #shells out all points to list of dicts
                    if len(dict_points) != 3:
                        raise Exception('A triangle needs to have exactly 3 points!')
                    points = []
                    for point in dict_points:
                        points.append(list(point.values()))   #convert list of dicts to list of lists [[x,y],[x,y],...]
                    dict_of_obstacles[obstacle_id] = Triangle(obstacle_id, center, rotation, points)
                elif obstacle_figure is _FigureType.RECTANGLE:
                    lengths = [figure['length_x'], figure['length_y']]
                    dict_of_obstacles[obstacle_id] = Rectangle(obstacle_id, center, rotation, lengths)
                elif obstacle_figure is _FigureType.CIRCLE:
                    radius = figure['radius']
                    dict_of_obstacles[obstacle_id] = Circle(obstacle_id, center, rotation, radius)
                break
        else:   #program runs into this else if it exits for loop normally (without break statement)
            raise Exception(f'Obstacle with id: {obstacle_id} cannot be found')
    return dict_of_obstacles

def modify_obstacle_position(obstacle_id: int, new_position)-> None:
    '''Modifies obstacle center and its rotation'''
    pass

class _FigureType(enum.Enum):
    TRIANGLE = 'triangle'
    RECTANGLE   = 'rectangle'
    CIRCLE   = 'circle'
    
if __name__ == '__main__':
    import json
    with open(".\Computer\\figures.json") as f:
        figures=json.load(f)['figures']
    with open(".\Computer\obstacles.json") as o:
        obstacles=json.load(o)['obstacles']
    dict_of_obstacles = load_obstacles(obstacles, figures)
    print(dict_of_obstacles[2].get_vertices())


