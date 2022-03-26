import numpy as np

class WorkingArea(object):
    def __init__(self, vertices: list[dict]) -> None:
        super().__init__()
        temp = []
        for vertice in vertices:
            temp.append(list(vertice['position'].values()))
        self._vertices = np.asarray(temp).transpose()
        self._x_max, self._y_max = np.max(self._vertices, axis = 1)
        self._x_min, self._y_min = np.min(self._vertices, axis = 1)
        return None

    def get_x_min(self) -> float:
        return self._x_min
    
    def get_x_max(self) -> float:
        return self._x_max
    
    def get_y_min(self) -> float:
        return self._y_min
    
    def get_y_max(self) -> float:
        return self._y_max

    def get_extremes(self) -> tuple[float]:
        return self._x_min, self._x_max, self._y_min, self._y_max
    
    def get_vertices(self) -> np.ndarray:
        return self._vertices



