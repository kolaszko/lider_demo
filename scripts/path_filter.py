import pickle
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import copy


Z_BORDER = 0.325374848408

def augment_list(path):
    new_path = []

    zero = True
    for p in path:
        print(p)
        if not zero:
            if p[2] <= 0.35:
                p_h = copy.copy(p)

                p_h[2] += 0.005
                new_path.append(p_h)
                new_path.append(p)
            else:
                new_path.append(p)
        else:
            new_path.append(p)
            zero = False

    return new_path

def augment_tf(path):
    new_path = []

    zero = True
    for p in path:
        if not zero:
            if p.pose.position.z <= 0.35:
                p_h = copy.deepcopy(p)

                p_h.pose.position.z += 0.003
                new_path.append(p_h)
                new_path.append(p)
            else:
                new_path.append(p)
        else:
            new_path.append(p)
            zero = False

    return new_path


if __name__ == '__main__':

    with open('curve_2_full.pickle', 'rb') as f:
        ds = pickle.load(f)

    path = []
    full_path = []
    vis = []

    last_coords = [0, 0, 0]

    for i in ds:
        t = [i.pose.position.x, i.pose.position.y, i.pose.position.z]
        d = np.linalg.norm(np.asarray(last_coords) - np.asarray(t))
        path.append(t)

        if d > 0.003:
            last_coords = t
            vis.append(t)
            full_path.append(i)

    path = np.asarray(path)

    vis = augment_list(vis)
    vis = np.asarray(vis)

    fp = augment_tf(full_path)
    full_path=fp

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(vis[:, 0], vis[:, 1], vis[:, 2])

    plt.show()

    with open('curve_2_full_run.pickle', 'wb') as f:
        pickle.dump(full_path, f)