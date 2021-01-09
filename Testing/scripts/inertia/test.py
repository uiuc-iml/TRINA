import pickle
import argparse
import sklearn.metrics
import numpy as np

from calibrate import num_inp_features, num_out_features


def main():
    parser = argparse.ArgumentParser(description="Train force predictor")
    parser.add_argument("--test", type=float, default=0.1,
        help="proprtion of data held out for testing")
    args = parser.parse_args()

    data_array = np.load('data.npy')
    np.random.shuffle(data_array)
    train_ind = int((1 - args.test) * len(data_array))
    X_test = data_array[train_ind:, :num_inp_features]
    y_test = data_array[train_ind:, num_inp_features:]
    with open("regressor.p", "rb") as of:
        regressor = pickle.load(of)
    predictions = regressor.predict(X_test)
    print("R2 score:", sklearn.metrics.r2_score(y_test, predictions))


if __name__ == "__main__":
    main()
