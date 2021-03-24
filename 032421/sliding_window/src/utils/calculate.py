import numpy as np

def get_linear_function(points, thresh_count=15):
    if len(points) < thresh_count:
        return False, None

    xs, ys = [], []

    for x, y in points:
        xs.append(x)
        ys.append(y)
    
    xs = np.array(xs)
    ys = np.array(ys)

    y0 = np.polyfit(ys, xs, 2)
    result_f = np.poly1d(y0)
    return True, result_f