import numpy as np

def decrease_function(angle, max_speed):
    func = np.poly1d([-0.01, 0, max_speed])
    return func(angle)


def decrease_function2(angle, max_speed):
    return max_speed * np.cos(angle / 80.0)


if __name__ == "__main__":  
    for i in range(-50, 51):
        for j in range(-50, 51):
            print("angle: {} | speed: {} | new_speed: {}".format(j, i, decrease_function2(j, i)))