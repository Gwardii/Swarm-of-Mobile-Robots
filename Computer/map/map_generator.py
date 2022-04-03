import numpy as np
from numpy import linalg as LA
from numpy.core.multiarray import empty
from map import working_area
from map import obstacles

class MapGenerator(object):
    def __init__(self, cell_size:float, working_area_obj: working_area.WorkingArea, obstacles: dict[obstacles.Obstacle]) -> None:
        super().__init__()
        self._cell_size = cell_size
        self._working_area = working_area_obj
        self._x_min, self._x_max, self._y_min, self._y_max = working_area_obj.get_extremes()
        x_space = np.arange(cell_size/2, self._x_max - self._x_min - cell_size/2, cell_size, dtype=float)
        y_space = np.arange(cell_size/2, self._y_max - self._y_min, cell_size, dtype=float) #may lose some working area
        self._cells_coordinates = MapGenerator._cartesian_product(x_space, y_space)
        self._max_cell_x = len(self._cells_coordinates) 
        self._max_cell_y = len(self._cells_coordinates[0])
        self._cells = set()
        for x_index in range(self._max_cell_x):
            for y_index in range(self._max_cell_y):
                self._cells.add((x_index, y_index))
        self._closed_cells = self._rasterize_borders()
        self._closed_cells = self._closed_cells.union(self._rasterize_obstacles(obstacles.values()))
        self._free_cells = self._cells.difference(self._closed_cells)
        self._distance_cells = self._distance_meter() ##dic{cell coordinates, closest neighbour relative coordinate, distance }
        # print(sorted(self._distance_cells.items()))
        return None

    def get_free_cells(self) -> dict:
        return self._free_cells

    def get_closed_cells(self) -> dict:
        return self._closed_cells

    def get_distance_cells(self):
        return self._distance_cells

    def _rasterize_borders(self) -> set:
        '''Rasterizes borders into node indexes which contain boundery of working area.\
            It is assumed that working area is a polygon'''
        vertices = self._working_area.get_vertices()
        vertices_relative = vertices - np.asarray([[self._x_min, self._y_min]]).transpose()
        vertices_relative_T = vertices_relative.transpose()
        x_min, y_min = np.min(vertices_relative_T, axis = 0)
        x_max, y_max = np.max(vertices_relative_T, axis = 0)
        def _point_not_in_polygon_with_borders(polygon, point):
            return MapGenerator._point_in_polygon(polygon, point, IN_POLYGON = False)
        closed_cells = self._rasterization(x_min, y_min, x_max, y_max, _point_not_in_polygon_with_borders, vertices_relative_T)
        return closed_cells

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

    def _rasterize_polygon(self, polygon: obstacles.Polygon) -> set:
        vertices_relative = polygon.get_vertices() - np.asarray([[self._x_min, self._y_min]]).transpose()
        vertices_relative_T = vertices_relative.transpose()
        x_min, y_min = np.min(vertices_relative_T, axis = 0)
        x_max, y_max = np.max(vertices_relative_T, axis = 0)
        cells = self._rasterization(x_min, y_min, x_max, y_max, MapGenerator._point_in_polygon, vertices_relative_T)
        return cells

    def _point_in_polygon(polygon: np.ndarray, point: list, IN_POLYGON = True, ON_EDGE = False) -> bool:
        odd = False
        i = -1
        j = len(polygon) - 1
        while i < len(polygon) - 1:
            i = i + 1
            side_vector = polygon[i] - polygon[j]
            point_vector = point - polygon[j]
            if side_vector[0] == 0:
                if point_vector[0] == 0:
                    if side_vector[1] == 0:
                        if point_vector[1] == 0:
                            return ON_EDGE
                    elif 0 <= (point_vector[1]/side_vector[1]) <= 1:
                        return ON_EDGE
            elif side_vector[1] == 0:
                if point_vector[1] == 0:
                    if 0 <= (point_vector[0]/side_vector[0]) <= 1:
                        return ON_EDGE
            else:
                aux_vector = point_vector / side_vector
                if aux_vector[0] == aux_vector[1]:
                    return ON_EDGE

            if (polygon[j][1] != polygon[i][1]):
                if (((polygon[i][1] > point[1]) != (polygon[j][1] > point[1])) and (point[0] < \
                    ((polygon[j][0] - polygon[i][0]) * (point[1] - polygon[i][1]) / (polygon[j][1] - polygon[i][1])) +polygon[i][0])):
                    odd = not odd
            j = i
        if odd:
            return IN_POLYGON
        else:
            return not IN_POLYGON

    def _rasterize_circle(self, circle: obstacles.Circle ) -> set:
        center = circle.get_center() - np.asarray([[self._x_min, self._y_min]])
        radius = circle.get_radius()
        x_min, y_min = (center[0][0] - radius, center[0][1] - radius)
        x_max, y_max = (center[0][0] + radius, center[0][1] + radius)
        cells = self._rasterization(x_min, y_min, x_max, y_max, MapGenerator._point_in_circle, (center, radius))
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
                        if j < self._max_cell_y:
                            boundary.append((i,j))
                        elif j > 0:
                            boundary.append((i, j - 1))
                    elif j == 0:
                        if i < self._max_cell_x:
                            boundary.append((i, j))
                        elif i > 0:
                            boundary.append((i - 1, j))
                    elif i == self._max_cell_x:
                        if j < self._max_cell_y:
                            boundary.append((i - 1, j))
                        elif i > 0:
                            boundary.append((i - 1, j - 1))
                    elif j == self._max_cell_y:
                        if i < self._max_cell_x:
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

    def _distance_meter(self) -> dict:
        def vector_norm(vector):
            norm = (vector[0]**2 + vector[1]**2)**.5
            return norm

        def check_neighbours(cell):
            current_cell = cells_distance[cell]
            neighbours = [(1, 0), (1, 1), (0, 1), (-1,1), (-1, 0), (-1,-1), (0, -1), (1, -1)]
            changed_cells = []
            for neighbour in neighbours:
                neighbour_cell = (cell[0] + neighbour[0], cell[1] + neighbour[1])
                if neighbour_cell not in self._free_cells:
                    continue
                neighbour_cell_distance = cells_distance[neighbour_cell]
                distance = [current_cell[0][0] + neighbour[0], current_cell[0][1] + neighbour[1]]
                distance_norm = vector_norm(distance)
                if neighbour_cell_distance[1] is None:
                    cells_distance[neighbour_cell] = [distance, distance_norm]
                    changed_cells.append([neighbour_cell, distance_norm])
                elif neighbour_cell_distance[1] > distance_norm:
                    cells_distance[neighbour_cell] = [distance, distance_norm]
                    changed_cells.append([neighbour_cell, distance_norm])
            return changed_cells

        cells_distance = {cell: [None, None] for cell in self._cells}
        for cell in self._closed_cells:
            cells_distance[cell] = [[0, 0], 0]
        cells_to_check = dict()
        for cell in self._free_cells:
            if cell[0] == 0:
                cells_distance[cell] = [[1, 0], 1]
                cells_to_check[cell] = 1
            if cell[0] == self._max_cell_x:
                cells_distance[cell] = [[-1, 0], 1]
                cells_to_check[cell] = 1
            if cell[1] == 0:
                cells_distance[cell] = [[0, 1], 1]
                cells_to_check[cell] = 1
            if cell[1] == self._max_cell_y:
                cells_distance[cell] = [[0, -1], 1]
                cells_to_check[cell] = 1
        for cell in self._closed_cells:
            _changed_cells = check_neighbours(cell)
            for _changed_cell in _changed_cells:
                cells_to_check[_changed_cell[0]] = _changed_cell[1]
        while len(cells_to_check) > 0:
            min_distance_norm = min(cells_to_check.values())
            cells_to_check_items = list(cells_to_check.items())
            for cell, _distance_norm in cells_to_check_items:
                if _distance_norm == min_distance_norm:
                    _changed_cells = check_neighbours(cell)
                    for _changed_cell in _changed_cells:
                        cells_to_check[_changed_cell[0]] = _changed_cell[1]
                    cells_to_check.pop(cell)
        return cells_distance






    