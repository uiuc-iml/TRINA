import pickle
import autosklearn.regression
import argparse
import numpy as np
import logging
import klampt


def main():
    parser = argparse.ArgumentParser(description="Learn model to predict "
        + "external force")
    parser.add_argument("inp", type=str, help="input file")
    parser.add_argument("-p", type=str, default="../../../Motion/data/",
        help="path to world file")
    parser.add_argument("-r", type=str, default="TRINA_world_cholera.urdf",
        help="world file name")
    parser.add_argument("-t", type=int, default=100,
        help="number of measurements to average to compute tear, use first "
        + "`tear_num` readings")
    args = parser.parse_args()

    logger = logging.getLogger(__name__)
    logger.setLevel(logging.WARNING)

    with open(args.inp, "rb") as inp:
        data = pickle.load(inp)
    tear_value = np.zeros(6)
    if args.t <= len(data):
        for i in range(args.t):
            tear_value += np.array(data[i]["wrench"])
        tear_value /= tear_num
    else:
        logger.warn(f"Skipped tear because tear_num={args.t} was greater "
            + "than the number of datapoints ({len(data)})")
    world = klampt.WorldModel()
    world.loadFile(args.p + args.r)
    robot = world.robot(0)


if __name__ == "__main__":
    main()
