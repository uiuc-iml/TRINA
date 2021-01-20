import pickle
import argparse
import sklearn.metrics
import numpy as np

from clean_data import num_inp_features, num_out_features


def main():
    parser = argparse.ArgumentParser(description="Test force predictor")
    parser.add_argument("--file", type=str, default="regressor.p",
        help="which regressor to test")
    args = parser.parse_args()

    test_data_array = np.load('test_data.npy')
    X_test = test_data_array[:, :num_inp_features]
    y_test = test_data_array[:, num_inp_features:]
    with open(args.file, "rb") as of:
        regressor = pickle.load(of)
    predictions = regressor.predict(X_test)
    print("R2 score:", sklearn.metrics.r2_score(y_test, predictions))


if __name__ == "__main__":
    main()
