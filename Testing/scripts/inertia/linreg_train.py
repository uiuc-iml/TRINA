import numpy as np
import pickle
import argparse
import sklearn.metrics

from clean_data import num_inp_features, num_out_features
from sklearn.linear_model import LinearRegression


def main():
    parser = argparse.ArgumentParser(description="Train force predictor")
    args = parser.parse_args()

    train_data_array = np.load('train_data.npy')
    test_data_array = np.load('test_data.npy')
    X_train = train_data_array[:, :num_inp_features]
    X_test = test_data_array[:, :num_inp_features]
    y_train = train_data_array[:, num_inp_features:]
    y_test = test_data_array[:, num_inp_features:]
    print(f"Using {len(X_train)} points for training")
    print(f"Using {len(X_test)} points for testing")
    regressor = LinearRegression()
    regressor.fit(X_train, y_train)
    with open("linear_regressor.p", "wb") as of:
        pickle.dump(regressor, of)
    predictions = regressor.predict(X_test)
    print(regressor.score(X_test, y_test))
    print("R2 score:", sklearn.metrics.r2_score(y_test, predictions))


if __name__ == "__main__":
    main()
