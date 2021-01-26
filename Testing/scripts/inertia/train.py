import numpy as np
import autosklearn.regression
import pickle
import argparse
import sklearn.metrics

from clean_data import num_inp_features, num_out_features


def main():
    parser = argparse.ArgumentParser(description="Train force predictor")
    args = parser.parse_args()

    train_data_array = np.load('train_data.npy')
    test_data_array = np.load('test_data.npy')
    X_train = train_data_array[:, :num_inp_features]
    X_test = test_data_array[:, :num_inp_features]
    y_train = train_data_array[:, num_inp_features:]
    y_test = test_data_array[:, num_inp_features:]
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
