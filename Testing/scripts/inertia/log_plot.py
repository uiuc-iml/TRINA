import pandas as pd
import json
import numpy as np
import matplotlib.pyplot as plt

from scipy.optimize import minimize
from scipy import signal as spsignal


def main():
    frame = pd.read_csv('../../../logs/2021_01_24_16_40_37_log', sep='|')
    robot_data = frame[frame['nature'] == 'robot']
    num_points = len(robot_data)
    print("Robot data points", num_points)
    b, a =spsignal.butter(6,0.01,'low')
    side = 'LeftArm' # or RightArm
    w = 'EEWrench'
    f = 'global'
    wrenches = np.empty((num_points, 6))
    for i, (idx, row) in enumerate(robot_data.iterrows()):
        data = json.loads(row.log.replace("'", '"'))
        wrenches[i] = data[w][side][f]
    # plt.plot(wrenches[:, :3])
    w_norm = np.linalg.norm(wrenches[:], axis=1)
    filtered_w = spsignal.lfilter(b, a, w_norm)
    deriv = wrenches[1:] - wrenches[:-1]
    d_norm = np.linalg.norm(deriv, axis=1)
    s_deriv = deriv[1:] - deriv[:-1]
    filtered_deriv = (filtered_w[1:] - filtered_w[:-1]) * 60
    # plt.plot(d_norm)
    # plt.plot(filtered_deriv)
    plt.plot(filtered_w)
    # plt.plot(w_norm)
    plt.show()


if __name__ == "__main__":
    main()
