import pickle
import argparse
import sklearn.metrics
import numpy as np
import matplotlib.pyplot as plt

from clean_data import num_inp_features, num_out_features


def main():
    parser = argparse.ArgumentParser(description="Test force predictor")
    parser.add_argument("sequ", type=str,
        help="chronologically recorded features file")
    parser.add_argument("--file", type=str, default="",
        help="which regressor to test")
    parser.add_argument("--title", type=str, default="", help="plot title")
    args = parser.parse_args()

    # test_data_array = np.load('test_data.npy')
    # X_test = test_data_array[:, :num_inp_features]
    # y_test = test_data_array[:, num_inp_features:]
    data = np.load(args.sequ)
    if args.file != "":
        with open(args.file, "rb") as of:
            regressor = pickle.load(of)
        pred = regressor.predict(data[:, :num_inp_features])
    # predictions = regressor.predict(X_test)
    # print("R2 score:", sklearn.metrics.r2_score(y_test, predictions))

    # plt.plot(data[:, -num_out_features:-3], label='GT')
    # plt.plot(pred[:, :3], label='Pred')
    plt.plot(data[:, -num_out_features:-3] - pred[:, :3], label="Corrected Ext Force")
    plt.legend()
    plt.title(args.title)
    plt.show()


if __name__ == "__main__":
    main()
