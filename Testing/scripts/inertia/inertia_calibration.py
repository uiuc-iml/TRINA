import sys
import time
import argparse
import numpy as np
import pickle
sys.path.append("/home/motion/TRINA")
from Motion import MotionClient
from Motion import TRINAConfig
from klampt.io.loader import load 
from klampt.model.trajectory import RobotTrajectory


def main():
	parser = argparse.ArgumentParser('collect inertia calibration data')
	parser.add_argument('--outfile', type=str, 
		default='inertia_record.p', help='output file name')
	args = parser.parse_args()
	mc = MotionClient()
	mc.startServer(mode = "Physical", components = ['right_limb','left_limb'], codename = 'bubonic')
	mc.startup()
	time.sleep(0.05)
	traj = load('auto', 'inertia.path')
	interval = 5
	interpolation_points = 10
	init_time = 10
	q_i = []
	data = []
	for t in TRINAConfig.get_left_active_Dofs('bubonic'):
		q_i.append(traj.milestones[0][t])
	mc.setLeftLimbPositionLinear(q_i, init_time)
	time.sleep(init_time)
	init_points = 100
	for i in range(init_points):
		collect_data(data, mc)
		time.sleep(init_time / init_points)
	#print("Setting velocity")
	#mc.setLeftLimbVelocity([1]*6)
	#time.sleep(init_time)
	for i, q in enumerate(traj.milestones):
		if i < len(traj.milestones) - 1:
			q_l = np.array(mc.sensedLeftLimbPosition())
			q_l_n = []
			for t in TRINAConfig.get_left_active_Dofs('bubonic'):
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
				collect_data(data, mc)
				time.sleep(interval / interpolation_points)
	with open(args.outfile, "wb") as of:
		pickle.dump(data, of)
	mc.shutdown()


def collect_data(collection, mc):
	p = mc.sensedLeftLimbPosition()
	v = mc.sensedLeftLimbVelocity()
	collection.append({
		'time': time.time(),
		'position': p,
		'velocity': v
	})


if __name__ == "__main__":
	main()
