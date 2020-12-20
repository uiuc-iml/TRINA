import sys
import time
sys.path.append("/home/motion/TRINA")
from Motion import MotionClient
from Motion import TRINAConfig
from klampt.io.loader import load 
from klampt.model.trajectory import RobotTrajectory


def main():
	mc = MotionClient()
	traj = load('auto', 'inertia.path')
	for i, q in enumerate(traj.milestones):
		q_l = []
		for t in TRINAConfig.get_left_active_Dofs('bubonic'):
			q_l.append(q[t])
		print(q_l)
		mc.setLeftLimbPositionLinear(q_l, 1.0)
		time.sleep(1)


if __name__ == "__main__":
	main()
