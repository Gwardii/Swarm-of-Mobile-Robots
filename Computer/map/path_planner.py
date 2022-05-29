import numpy as np
from map import map_generator
import robot_handler

class PathPlanner(object):
    def __init__(self, raster_map: map_generator.MapGenerator = None, robots: dict[int, robot_handler.Robot] = dict(), margin = 0) -> None:
        super().__init__()
        self._map = raster_map
        self._robots = robots
        self._margin = margin
        self._paths=None
        return None

    def change_map(self, new_map):
        self._map = new_map
        return None
    
    def add_robot(self, robot_id: int, robot: robot_handler.Robot) -> None:
        self._robots[robot_id] = robot
        return None

    def pop_robot(self, robot_id: int) -> robot_handler.Robot:
        robot = self._robots.pop(robot_id)
        return robot

    def set_robots_targets(self, robots_id_with_targets: list[tuple[int, list[float]]]) -> list[int]:
        robots_not_in_planner = list()
        for robot_id, target in robots_id_with_targets:
            if robot_id in self._robots:
                self._robots[robot_id].set_target(target)
            else:
                robots_not_in_planner.append(robot_id)
        return robots_not_in_planner

    def get_paths(self):
        return self._paths

    def _determine_paths(self) -> bool:
        unreachable_target_robots = list()
        for robot_id, robot in self._robots.items():
            self.rated_cells, is_target_reachable = self._rate_cells(robot)
            self._find_path(robot, self.rated_cells)

    def _rasterize_line_between_cells(self, cell_1, cell_2):
        def my_round(x):
            n = int(x)
            if(abs(x) % 1 > 0.5):
                return n + x/abs(x)
            return n
        def my_round_2(x):
            n = int(x)
            if(abs(x) % 1 < 0.5):
                return n
            return n + x/abs(x)
        x = cell_2[0] - cell_1[0]
        y = cell_2[1] - cell_1[1]
        if(x == 0):
            y_sign = int(y/abs(y))
            rasterized_line = {(cell_2[0], _y) for _y in range(cell_1[1],cell_2[1]+y_sign, y_sign)}
        elif(y == 0):
            x_sign = int(x/abs(x))
            rasterized_line = {(_x, cell_2[1]) for _x in range(cell_1[0],cell_2[0]+x_sign, x_sign)}
        elif(abs(y) > abs(x)):
            x_sign = int(x/abs(x))
            y_sign = int(y/abs(y))
            y_x = y/x
            _y0 = cell_1[1]
            rasterized_line = set()
            for i, _dx in enumerate(np.arange(0, abs(x), 0.5)):
                _x = cell_1[0] + int(np.ceil(_dx)*x_sign)
                _y1 = int(_y0 + my_round_2(0.5*i*y_x*x_sign))
                _y2 = int(_y0 + my_round(0.5*(i+1)*y_x*x_sign))
                rasterized_line.update({(_x, _y) for _y in range(_y1,_y2+x_sign, x_sign)})
        else:
            x_sign = int(x/abs(x))
            y_sign = int(y/abs(y))
            x_y = x/y
            _x0 = cell_1[0]
            rasterized_line = set()
            for i, _dy in enumerate(np.arange(0, abs(y), 0.5)):
                _y = cell_1[1] + int(np.ceil(_dy)*y_sign)
                _x1 = int(_x0 + my_round_2(0.5*i*x_y*y_sign))
                _x2 = int(_x0 + my_round(0.5*(i+1)*x_y)*y_sign)
                rasterized_line.update({(_x, _y) for _x in range(_x1,_x2+x_sign, x_sign)})
        return rasterized_line

    def _rate_cells(self, robot):
        def neighbours(cell):
            _prime_neighbours = set()
            _aux_prime_meighbours = set()
            _diagonal_neighbours = set()
            _PRIME_NEIGHBOURS = [(1, 0), (0, 1), (-1, 0), (0, -1)]
            _DIAGONAL_NEIGHBOURS = [(1, 1), (-1, 1), (-1, -1), (1, -1)]
            i = -1
            for _neighbour in _PRIME_NEIGHBOURS:
                _neighbour_cell = self._add_tuples(cell, _neighbour)
                if _neighbour_cell in available_cells:
                    _prime_neighbours.add(_neighbour_cell)
                    _aux_prime_meighbours.add(_neighbour)
            for _neighbour in _DIAGONAL_NEIGHBOURS:
                if (_neighbour[0], 0) in _aux_prime_meighbours and (0, _neighbour[1]) in _aux_prime_meighbours:
                    _neighbour_cell = self._add_tuples(cell, _neighbour)
                    if _neighbour_cell in available_cells:
                        _diagonal_neighbours.add(_neighbour_cell)
            return _prime_neighbours, _diagonal_neighbours

        def rate_cells(parent_rate, _prime_neighbours, _diagonal_neighbours):
            for cell in _prime_neighbours:
                rated_cells[cell] = parent_rate - prime_step
                available_cells.remove(cell)
            for cell in _diagonal_neighbours:
                rated_cells[cell] = parent_rate - diagonal_step
                available_cells.remove(cell)

        max_potential = len(self._map._distance_cells)
        prime_step = 2
        diagonal_step = 3
        rated_cells = dict()
        cell_size = self._map._cell_size
        min_distance = robot._radius / (cell_size + self._margin)
        available_cells = {cell for cell, distance in self._map._distance_cells.items() if distance[1] >= min_distance}
        target_cell = self._position_to_cell(robot._target[0])
        is_target_reachable = target_cell in available_cells
        if is_target_reachable:
            rated_cells[target_cell] = max_potential
            available_cells.remove(target_cell)
            U = {target_cell}
            V = set()
            while bool(U):
                for cell in U:
                    prime_cells, diagonal_cells = neighbours(cell)
                    rate_cells(rated_cells[cell], prime_cells, diagonal_cells)
                    V.update(prime_cells)
                    V.update(diagonal_cells)
                U = V.copy()
                V.clear()
        return rated_cells, is_target_reachable

    def _find_path(self, robot, rated_cells):
        _NEIGHBOURS = [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, 1), (-1, -1), (1, -1)]
        class Node():
            def __init__(self, cell = None, prev_node = None) -> None:
                self._next_nodes = None
                self._prev_node = prev_node
                self._cell = cell
                return None
        class PathTree():
            def __init__(self, first_node) -> None:
                self._first_node = first_node
                self._last_nodes = [first_node]
                return None
            def _add_nodes(self, nodes):
                nodes[0]._prev_node._next_nodes = nodes
                self._last_nodes.remove(nodes[0]._prev_node)
                self._last_nodes.extend(nodes)
                return None
            def _to_list(self, node):
                _cells_list = [node._cell]
                while node._prev_node is not None:
                    node = node._prev_node
                    _cells_list.append(node._cell)
                _cells_list.reverse()
                return _cells_list


        starting_cell = self._position_to_cell(robot.get_coordinates()[0])
        target_cell = self._position_to_cell(robot._target[0])
        line = self._rasterize_line_between_cells(starting_cell, target_cell)
        if(all([cell in rated_cells for cell in line])):
            robot.path = {starting_cell, target_cell}
            return
        paths = PathTree(Node(starting_cell, None))
        active_nodes = paths._last_nodes.copy()
        while bool(active_nodes):
            next_active_nodes = list()
            for node in active_nodes:
                cell = node._cell
                neighbours = [self._add_tuples(cell, _neighbour) for _neighbour in _NEIGHBOURS]
                neighbours = [neighbour for neighbour in neighbours if neighbour in rated_cells]
                if bool(neighbours):
                    ratings = [rated_cells[neighbour] for neighbour in neighbours]
                    max_rating = max(ratings)
                    if max_rating >= rated_cells[node._cell]:
                        _nodes_to_add = [Node(neighbour, node) for neighbour in neighbours if rated_cells[neighbour] == max_rating]
                        paths._add_nodes(_nodes_to_add)
                        next_active_nodes.extend(_nodes_to_add)
            active_nodes = next_active_nodes
            set_of_paths = set()
        for node in paths._last_nodes:
            path = paths._to_list(node)
            path = self._minimize_path(path)
            set_of_paths.add(path)
        robot.path = list(set_of_paths)[0]

    def _minimize_path(self, path):
        size = len(path)
        if(size < 3):
            new_path = path
        else:
            cell_1 = path[0]
            cell_3 = path[-1]
            line = self._rasterize_line_between_cells(cell_1, cell_3)
            if(all([cell in self.rated_cells for cell in line])):
                new_path = [cell_1, cell_3]
            else:
                cell_2 = path[int(size/2)]
                new_path = self._minimize_path([cell_1, cell_2])[0:-1]
                new_path.append(self._minimize_path([cell_2, cell_3]))
        return new_path


    def _position_to_cell(self, position):
        x = int(position[0] / self._map._cell_size)
        y = int(position[1] / self._map._cell_size)
        return (x, y)

    def _add_tuples(self, tuple_1, tuple_2):
        aux_list = list()
        for i in range(len(tuple_1)):
            aux_list.append(tuple_1[i] + tuple_2[i])
        return tuple(aux_list)

    def get_rated_cells(self, robot):
        return self._rate_cells(robot)






