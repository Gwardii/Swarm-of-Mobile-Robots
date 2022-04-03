from map import map_generator
import robot_handler

class PathPlanner(object):
    def __init__(self, raster_map: map_generator.MapGenerator = None, robots: dict[int, robot_handler.Robot] = None, margin = 0) -> None:
        super().__init__()
        self._map = raster_map
        self._robots = robots
        self._margin = margin

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
            rated_cells, is_target_reachable = self._rate_cells(robot)

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
        target_cell = self._position_to_cell(robot._position)
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
            def _add_nodes(self, cells, prev_node):
                prev_node._next_nodes = [Node(cell, prev_node) for cell in cells]
                self._last_nodes.remove(prev_node)
                self._last_nodes.extend(prev.node._next_nodes)
                return None

        starting_cell = self._position_to_cell(robot.get_coordinates()[0])
        paths = PathTree(Node(starting_cell, None))
        active_nodes = paths._last_nodes
        while bool(active_nodes):
            for node in active_nodes:
                pass




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






