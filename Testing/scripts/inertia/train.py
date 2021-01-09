import numpy as np
import autosklearn.regression
import pickle
import argparse
import sklearn.metrics

from calibrate import num_inp_features, num_out_features


def main():
    parser = argparse.ArgumentParser(description="Train force predictor")
    parser.add_argument("--test", type=float, default=0.1,
        help="proprtion of data held out for testing")
    args = parser.parse_args()

    data_array = np.load('data.npy')
    np.random.shuffle(data_array)
    train_ind = int((1 - args.test) * len(data_array))
    X_train = data_array[:train_ind, :num_inp_features]
    X_test = data_array[train_ind:, :num_inp_features]
    y_train = data_array[:train_ind, num_inp_features:]
    y_test = data_array[train_ind:, num_inp_features:]
    regressor = autosklearn.regression.AutoSklearnRegressor(
        time_left_for_this_task=150,
        per_run_time_limit=30
    )
    regressor.fit(X_train, y_train)
    print(regressor.show_models())
    with open("regressor.p", "wb") as of:
        pickle.dump(regressor, of)
    predictions = regressor.predict(X_test)
    print("R2 score:", sklearn.metrics.r2_score(y_test, predictions))


if __name__ == "__main__":
    main()
