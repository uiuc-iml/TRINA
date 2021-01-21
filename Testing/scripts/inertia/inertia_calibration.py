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


def main():
	parser = argparse.ArgumentParser('collect inertia calibration data')
	parser.add_argument('--outfile', type=str,
		default='inertia_record.p', help='output file name')
	args = parser.parse_args()
	mc = MotionClient()
	mc.startServer(mode=MODE, components=COMPONENTS, codename=CODENAME)
	mc.startup()
	time.sleep(0.05)
	traj = load('auto', 'cholera.path')
	interval = 5
	interpolation_points = 10
	init_time = 10
	q_i = []
	data = []
	for i, t in enumerate(TRINAConfig.get_left_active_Dofs(CODENAME)):
		# q_i.append(traj.milestones[0][t])
		q_i.append(TRINAConfig.left_untucked_config[i])
	mc.setLeftLimbPositionLinear(q_i, init_time)
	time.sleep(init_time)
	init_points = 100
	for i in range(init_points):
		collect_data(data, mc)
		time.sleep(init_time / init_points)
	#print("Setting velocity")
	#mc.setLeftLimbVelocity([1]*6)
	#time.sleep(init_time)
	collection_thread = threading.Thread(target=data_collection_thread,
		daemon=True)
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
					interval / interpolation_points)
				time.sleep(interval / interpolation_points)
	C_FLAG = False
	collection_thread.join()
	with open(args.outfile, "wb") as of:
		pickle.dump(data, of)
	mc.shutdown()


def data_collection_thread(collection):
	# Second motion client that only reads -> don't need to worry about locking
	# because it will never send a command that conflicts with the other
	# MotionClient
	mc = MotionClient()
	mc.startServer(mode=MODE, components=COMPONENTS, codename=CODENAME)
	mc.startup()
	time.sleep(0.05)
	while(C_FLAG):
		collect_data(collection, mc)
		time.sleep(1/20)


def collect_data(collection, mc):
	p = mc.sensedLeftLimbPosition()
	v = mc.sensedLeftLimbVelocity()
	w = mc.sensedLeftEEWrench(frame='local')
	collection.append({
		'time': time.time(),
		'position': p,
		'velocity': v,
		'wrench': w
	})


if __name__ == "__main__":
	main()
