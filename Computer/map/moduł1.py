import numpy as np

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

print(_point_in_polygon(np.array([[   0,    0], [2000,    0], [2000, 1600], [0, 1600]]), [2000,0], ON_EDGE = True))
