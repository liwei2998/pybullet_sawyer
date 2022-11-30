import numpy as np
import pybullet as p
import time
import math
from datetime import datetime
import util
####################connection
p.connect(p.GUI)
# p.connect(p.DIRECT)

###################model initialization
p.loadURDF("plane.urdf",[0,0,0])
humanId = p.loadURDF("sawyer_robot/sawyer_description/urdf/human166.urdf",[0,0,0])
humanpose = p.getBasePositionAndOrientation(humanId)
p.resetBasePositionAndOrientation(humanId,[1.8,0,0.95],p.getQuaternionFromEuler([0,0,3.14])) # x0_human=0.8
sawyerId = p.loadURDF("sawyer_robot/sawyer_description/urdf/sawyer.urdf",[-0.4,0,0.91488]) # x0_robot=-0.4

###################robot joint information
# Sawyer active joints id: 3,4,8,9,10,11,13,16(eef)
sawyerEndEffectorIndex = 16
n_joints_sawyer = p.getNumJoints(sawyerId)
joint_dict_sawyer={'right_j0': 3, 'head_pan': 4, 'right_j1': 8, 'right_j2': 9, \
				'right_j3': 10, 'right_j4': 11, 'right_j5': 13, 'right_j6': 16}
n_joints_human = p.getNumJoints(humanId)

#####################visualization and calculate inverse kinematics
joint_dict = util.get_joint_info(humanId,n_joints_human)
print ('\n###############################\n',joint_dict)
vis = util.visualization(humanId)
vis.vis_local_frame([28,36],True)
# vis.add_debug_param(n_joints_human)
# #####################human body ik
pos = [[0.6,-0.3,0.],[0.6,0.3,0.]]
eef_hum = [21,30] # right-hand, left-hand 
curr_joint_pos = list(np.array(p.getJointStates(humanId,np.array([*joint_dict.values()])[:,0]))[:,0])
jointPoses = p.calculateInverseKinematics2(humanId,eef_hum,pos,currentPositions=curr_joint_pos)
print ('\n invese unselected',jointPoses,type(jointPoses))
# vis.vis_one_pos(n_joints_human,jointPoses)
# vis.vis_static(20)
ik = util.get_ik(humanId,8.78)
cvae_pos = list(np.array([36.8,12.5,0.0,47.2,66.1,0.0,47.2,66.1,2.4,23.2,2.4,23.2])*np.pi/180)
jointPoses = ik.accurateIK(eef_hum,pos,joint_dict,curr_joint_pos,cvae_pos)
print ('\n invese selected',jointPoses,type(jointPoses))
vis.vis_one_pos(n_joints_human,jointPoses)
vis.vis_static(500)
# # Human activate joints id: 0,1,4,5,7,8,10,11,13,14
# 

# joint_dict_human={'upperbody': 0, 'neck': 1, 'right_shoulder': 4, 'right_elbow': 5, \
# 				'left_shoulder': 7, 'left_elbow': 8, 'right_hip': 10, 'right_knee': 11,\
# 				'left_hip': 13, 'left_knee': 14}


# Set eef pos for human
# pos = [[0.6,0,-0.6],[0.6,0,0.3]]
# eef_hum = [6,9] # right-hand, left-hand 
# curr_joint_pos = list(np.array(p.getJointStates(humanId,joint_id))[:,0])
# jointPoses = p.calculateInverseKinematics2(humanId,eef_hum,pos,currentPositions=curr_joint_pos)
# print ('\n invese',jointPoses,type(jointPoses))
# visOnePose(jointAngle=jointPoses)
# # Reset joint states
# for i in range (numJoints):
# 	jointInfo = p.getJointInfo(sawyerId, i)
# 	qIndex = jointInfo[3]
# 	if qIndex > -1:
# 		p.resetJointState(sawyerId,i,jointPoses[qIndex-7])

# while 1:
# 	a=1
		# jointInfo = p.getJointInfo(sawyerId, i)
		# qIndex = jointInfo[3]
		# if qIndex > -1:
		# 	joint_name = jointInfo[1]
		# 	p.addUserDebugParameter(joint_name.decode('utf-8'),-2.9842,2.9842,jointPoses[qIndex-7])
		# 	p.resetJointState(sawyerId,i,jointPoses[qIndex-7])

# ##########################################calculate inverse kinetics, save for later
# p.setGravity(0,0,0)
# t=0.
# prevPose=[0,0,0]
# prevPose1=[0,0,0]
# hasPrevPose = 0

# useRealTimeSimulation = 0
# p.setRealTimeSimulation(useRealTimeSimulation)
# #trailDuration is duration (in seconds) after debug lines will be removed automatically
# #use 0 for no-removal
# trailDuration = 15
	
# while 1:
# 	if (useRealTimeSimulation):
# 		dt = datetime.now()
# 		t = (dt.second/60.)*2.*math.pi
# 		print (t)
# 	else:
# 		t=t+0.01
# 		time.sleep(0.01)
	
# 	for i in range (1):
# 		pos = [1.0,0.2*math.cos(t),0.+0.2*math.sin(t)]
# 		jointPoses = p.calculateInverseKinematics(sawyerId,sawyerEndEffectorIndex,pos,jointDamping=jd)

# 		#reset the joint state (ignoring all dynamics, not recommended to use during simulation)
# 		for i in range (numJoints):
# 			jointInfo = p.getJointInfo(sawyerId, i)
# 			qIndex = jointInfo[3]
# 			if qIndex > -1:
# 				p.resetJointState(sawyerId,i,jointPoses[qIndex-7])

# 	ls = p.getLinkState(sawyerId,sawyerEndEffectorIndex)
# 	if (hasPrevPose):
# 		p.addUserDebugLine(prevPose,pos,[0,0,0.3],1,trailDuration)
# 		p.addUserDebugLine(prevPose1,ls[4],[1,0,0],1,trailDuration)
# 	prevPose=pos
# 	prevPose1=ls[4]
# 	hasPrevPose = 1		
