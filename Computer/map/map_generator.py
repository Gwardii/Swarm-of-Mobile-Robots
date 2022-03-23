import numpy as np
from numpy import linalg as LA
from working_area import WorkingArea
import obstacles

class MapGenerator(object):
    def __init__(self, cell_size:float, working_area: WorkingArea, obstacles: list[obstacles.Obstacle]) -> None:
        super().__init__()
        self._cell_size = cell_size
        self._working_area = working_area
        self._x_min, self._x_max, self._y_min, self._y_max = working_area.get_extremes()
        x_space = np.arange(cell_size/2, self._x_max - self._x_min, cell_size, dtype=float)
        y_space = np.arange(cell_size/2, self._y_max - self._y_min, cell_size, dtype=float) #may lose some working area
        self.cells = MapGenerator._cartesian_product(x_space, y_space)
        self._max_node_x = len(self.cells) - 1 
        self._max_node_y = len(self.cells[0]) - 1
        self._free_cells = self._rasterize_borders()
        self._closed_cells = self._rasterize_obstacles()
        self._free_cells = self._free_cells.difference(self._closed_cells)

        return None

    def _rasterize_borders(self) -> set:
        '''Rasterizes borders into node indexes which contain boundery of working area.\
            It is assumed that working area is a polygon'''
        vertices = self._working_area.get_vertices()
        vertices_relative = vertices - np.asarray([[self._x_min, self._y_min]]).transpose()
        vertices_relative_T = vertices_relative.transpose()
        x_min, y_min = np.min(vertices_relative_T, axis = 0)
        x_max, y_max = np.max(vertices_relative_T, axis = 0)
        point_not_in_polygon = lambda a1, a2: not self._point_in_polygon(a1, a2)
        free_cells = self._rasterization(x_min, y_min, x_max, y_max, point_not_in_polygon, vertices_relative_T)
        return free_cells

    def _rasterize_obstacles(self, list_of_obstacles: list[obstacles.Obstacle]) -> set:
        closed_cells = set([])
        for obstacle in list_of_obstacles:
            if isinstance(obstacle, obstacles.Circle):
                closed_cells = closed_cells.union(self._rasterize_circle(obstacle))
            elif isinstance(obstacle, obstacles.Polygon):
                closed_cells = closed_cells.union(self._rasterize_polygon(obstacle))
            else:
                raise Exception(f'Rasterization of {type(obstacle)} is not implemented')
        return closed_cells

    def _rasterize_polygon(self, vertices: np.ndarray) -> set:
        vertices_relative = vertices - np.asarray([[self._x_min, self._y_min]]).transpose()
        vertices_relative_T = vertices_relative.transpose()
        x_min, y_min = np.min(vertices_relative_T, axis = 0)
        x_max, y_max = np.max(vertices_relative_T, axis = 0)
        cells = self._rasterization(x_min, y_min, x_max, y_max, self._point_in_polygon, vertices_relative_T)
        return cells

    def _point_in_polygon(polygon: np.ndarray, point: np.ndarray) -> bool:
        """
        Raycasting Algorithm to find out whether a point is in a given polygon.
        Performs the even-odd-rule Algorithm to find out whether a point is in a given polygon.
        This runs in O(n) where n is the number of edges of the polygon.
         *
        :param polygon: an array representation of the polygon where polygon[i][0] is the x Value of the i-th point and polygon[i][1] is the y Value.
        :param point:   an array representation of the point where point[0] is its x Value and point[1] is its y Value
        :return: whether the point is in the polygon (not on the edge, just turn < into <= and > into >= for that)
        """

        # A point is in a polygon if a line from the point to infinity crosses the polygon an odd number of times
        odd = False
        # For each edge (In this case for each point of the polygon and the previous one)
        i = -1
        j = len(polygon) - 1
        while i < len(polygon) - 1:
            i = i + 1
            # If a line from the point into infinity crosses this edge
            # One point needs to be above, one below our y coordinate
            # ...and the edge doesn't cross our Y corrdinate before our x coordinate (but between our x coordinate and infinity)

            if (((polygon[i][1] > point[1]) != (polygon[j][1] > point[1])) and (point[0] < (
                    (polygon[j][0] - polygon[i][0]) * (point[1] - polygon[i][1]) / (polygon[j][1] - polygon[i][1])) +
                                                                                polygon[i][0])):
                # Invert odd
                odd = not odd
            j = i
        # If the number of crossings was odd, the point is in the polygon
        return odd

    def _rasterize_circle(self, circle: obstacles.Circle ) -> set:
        center = circle.get_center() - np.asarray([[self._x_min, self._y_min]])
        radius = circle.get_radius()
        x_min, y_min = center - radius
        x_max, y_max = center + radius
        cells = self._rasterization(x_min, y_min, x_max, y_max, self._point_in_circle, radius)
        return cells


    def _point_in_circle(circle: tuple, point: np.ndarray) -> bool:
        center = circle[0]
        radius = circle[1]
        distance_vector = center - point
        in_circle = LA.norm(distance_vector) < radius
        return in_circle
        

    def _rasterization(self,x_min, y_min, x_max, y_max, function, argument) -> set:
        low_x_index = int(x_min / self._cell_size)
        low_y_index = int(y_min / self._cell_size)
        high_x_index = int(x_max / self._cell_size)
        high_y_index = int(y_max / self._cell_size)
        boundary = []
        for i in range(low_x_index, high_x_index + 1):
            for j in range(low_y_index, high_y_index + 1):
                point = [i * self._cell_size, j * self._cell_size]
                if function(argument, point):
                    if i == 0:
                        if j < self._max_node_y:
                            boundary.append((i,j))
                        elif j > 0:
                            boundary.append((i, j - 1))
                    elif j == 0:
                        if i < self._max_node_x:
                            boundary.append((i, j))
                        elif i > 0:
                            boundary.append((i - 1, j))
                    elif i == self._max_node_x:
                        if j < self._max_node_y:
                            boundary.append((i - 1, j))
                        elif i > 0:
                            boundary.append((i - 1, j - 1))
                    elif j == self._max_node_y:
                        if i < self._max_node_x:
                            boundary.append((i, j - 1))
                        elif i > 0:
                            boundary.append((i - 1, j - 1))
                    else:
                        boundary.append((i,j))
                        boundary.append((i,j - 1))
                        boundary.append((i - 1,j))
                        boundary.append((i - 1,j - 1))
        return set(boundary)

    def _cartesian_product(x_space: np.ndarray, y_space: np.ndarray) -> list:
        product = []
        for x in x_space:
            partial_product = []
            for y in y_space:
                partial_product.append((x,y))
            product.append(partial_product)
        return product


    if __name__ == '__main__':
        print(_point_in_polygon(np.array([[0,0],[0,1],[1,0]]), np.array([-.2,.2])))