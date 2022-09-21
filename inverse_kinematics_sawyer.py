import numpy as np
import pybullet as p
import time
import math
from datetime import datetime

p.connect(p.GUI)
# p.connect(p.DIRECT)

# Load models and reset their bases
p.loadURDF("plane.urdf",[0,0,0])
humanId = p.loadURDF("sawyer_robot/sawyer_description/urdf/human.urdf",[0,0,0.])
humanpose = p.getBasePositionAndOrientation(humanId)
p.resetBasePositionAndOrientation(humanId,[0.8,0,0.85],p.getQuaternionFromEuler([1.57,0,3.14])) # x0_human=0.8
sawyerId = p.loadURDF("sawyer_robot/sawyer_description/urdf/sawyer.urdf",[-0.4,0,0.91488]) # x0_robot=-0.4

# Sawyer active joints id: 3,4,8,9,10,11,13,16(eef)
sawyerEndEffectorIndex = 16
n_joints_sawyer = p.getNumJoints(sawyerId)
# Human activate joints id: 0,1,4,5,7,8,10,11,13,14
n_joints_human = p.getNumJoints(humanId)

# # Get joints information
# for i in range(n_joints_human):
# 	infoJoints = p.getJointInfo(humanId,i)
# 	print ('\njoint information',infoJoints[0],infoJoints[1],infoJoints[2],infoJoints[3])
# 	joint_dict_human[str(infoJoints[1].decode('utf-8'))] = int(infoJoints[0])

joint_dict_human={'upperbody': 0, 'neck': 1, 'right_shoulder': 4, 'right_elbow': 5, \
				  'left_shoulder': 7, 'left_elbow': 8, 'right_hip': 10, 'right_knee': 11,\
				  'left_hip': 13, 'left_knee': 14}
 
joint_dict_sawyer={'right_j0': 3, 'head_pan': 4, 'right_j1': 8, 'right_j2': 9, \
				   'right_j3': 10, 'right_j4': 11, 'right_j5': 13, 'right_j6': 16}

start = 0

def visOnePose(jointAngle=np.array([0,-30,80,30,80,30,20,-20,20,-20])*np.pi/180,t=5):
	count = 0
	for joint in joint_dict_human.keys():
		p.resetJointState(humanId,joint_dict_human[joint],jointAngle[count])
		count += 1
	foot = p.getLinkState(humanId,15)
	p.resetBasePositionAndOrientation(humanId,[0.8,0,0.85-foot[0][2]+0.025],p.getQuaternionFromEuler([1.57,0,3.14]))

	global start
	while start <= t:
		start += 0.1
		time.sleep(0.1)

def visMultiPoses(jointAngle,n_poses=10,t=5):
	for i in range(n_poses):
		count = 0
		for joint in joint_dict_human.keys():
			p.resetJointState(humanId,joint_dict_human[joint],jointAngle[i][count])
			count += 1
		foot = p.getLinkState(humanId,15)
		p.resetBasePositionAndOrientation(humanId,[0.8,0,0.85-foot[0][2]+0.025],p.getQuaternionFromEuler([1.57,0,3.14]))

		global start
		while start <= t:
			start += 0.1
			time.sleep(0.1)		

visOnePose()
# #joint damping coefficents
# jd=[0.001]*numJoints

# # Set eef pos for sawyer
# pos = [0.5,0.,1.6]
# jointPoses = p.calculateInverseKinematics(sawyerId,sawyerEndEffectorIndex,pos,jointDamping=jd)

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
