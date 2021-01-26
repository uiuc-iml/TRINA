import sys
import time
import argparse
import numpy as np
import pickle
import threading
sys.path.append("/home/motion/TRINA")
from Motion import MotionClient
from Motion import TRINAConfig
from klampt.io.loader import load
from klampt.model.trajectory import RobotTrajectory

CODENAME = 'cholera'
COMPONENTS = ['left_limb']
MODE = "Physical"
C_FLAG = True
COL_CHECK = False


def main():
	global C_FLAG
	parser = argparse.ArgumentParser('collect inertia calibration data')
	parser.add_argument('--traj', type=str,
		default='cholera.path', help='input trajectory')
	parser.add_argument('--outfile', type=str,
		default='inertia_record.p', help='output file name')
	args = parser.parse_args()
	mc = MotionClient()
	mc.startServer(mode=MODE, components=COMPONENTS, codename=CODENAME)
	mc.startup()
	time.sleep(0.05)
	traj = load('auto', args.traj)
	interval = 5
	interpolation_points = 10
	init_time = 3
	q_i = []
	data = []
	for i, t in enumerate(TRINAConfig.get_left_active_Dofs(CODENAME)):
		q_i.append(TRINAConfig.left_untucked_config[i])
	print("Sending left arm to home position")
	mc.setLeftLimbPositionLinear(q_i, init_time, col_check=COL_CHECK)
	time.sleep(init_time + 0.3)
	print("Reached home position, recording wrench measurements for tare")
	init_points = 100
	for i in range(init_points):
		collect_data(data, mc)
		time.sleep(init_time / init_points)
	print("Starting recorded trajectory")
	collection_thread = threading.Thread(target=data_collection_thread,
		daemon=True, args=(data,))
	collection_thread.start()
	for i, q in enumerate(traj.milestones):
		if i < len(traj.milestones) - 1:
			q_l = np.array(mc.sensedLeftLimbPosition())
			q_l_n = []
			for t in TRINAConfig.get_left_active_Dofs(CODENAME):
				q_l_n.append(traj.milestones[i+1][t])
			q_l_n = np.array(q_l_n)
			for j in range(interpolation_points):
				#print("Q_L", q_l)
				#v = (q_l_n - q_l) * (j + 1) / interpolation_points
				#print(np.linalg.norm(v), v)
				#mc.setLeftLimbVelocity(v.tolist())
				q_t = ((q_l_n - q_l) * (j/interpolation_points)) + q_l
				print(q_t)
				mc.setLeftLimbPositionLinear(
					q_t.tolist(),
					interval / interpolation_points, col_check=COL_CHECK)
				time.sleep(interval / interpolation_points)
	print("Done with motion, saving")
	C_FLAG = False
	collection_thread.join()
	with open(args.outfile, "wb") as of:
		pickle.dump(data, of)
	print("Last data point: ", data[-1])
	mc.shutdown()


def data_collection_thread(collection):
	global C_FLAG
	# Second motion client that only reads -> don't need to worry about locking
	# because it will never send a command that conflicts with the other
	# MotionClient
	mc = MotionClient()
	mc.startServer(mode=MODE, components=COMPONENTS, codename=CODENAME)
	mc.startup()
	time.sleep(0.05)
	while(C_FLAG):
		collect_data(collection, mc)
		time.sleep(1/300)


def collect_data(collection, mc):
	p = mc.sensedLeftLimbPosition()
	v = mc.sensedLeftLimbVelocity()
	w = mc.sensedLeftEEWrench()
	collection.append({
		'time': time.time(),
		'position': p,
		'velocity': v,
		'wrench': w
	})


if __name__ == "__main__":
	main()
