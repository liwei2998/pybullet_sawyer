import pybullet as p
import time
import math
from datetime import datetime

p.connect(p.GUI)
p.loadURDF("plane.urdf",[0,0,0])
sawyerId = p.loadURDF("sawyer_robot/sawyer_description/urdf/sawyer.urdf",[-0.4,0,0.91488]) # actually the base is at x=-0.5

#bad, get it from name! sawyerEndEffectorIndex = 18
sawyerEndEffectorIndex = 16
numJoints = p.getNumJoints(sawyerId)
# Sawyer active joints id: 3,4,8,9,10,11,16
for i in range(numJoints):
	infoJoints = p.getJointInfo(sawyerId,i)
	print ('\njoint information',infoJoints[0],infoJoints[1],infoJoints[2],infoJoints[3])

#joint damping coefficents
jd=[0.001]*numJoints

# Set eef pos for sawyer
pos = [0.5,0.,1.6]
jointPoses = p.calculateInverseKinematics(sawyerId,sawyerEndEffectorIndex,pos,jointDamping=jd)

# Reset joint states
for i in range (numJoints):
	jointInfo = p.getJointInfo(sawyerId, i)
	qIndex = jointInfo[3]
	if qIndex > -1:
		p.resetJointState(sawyerId,i,jointPoses[qIndex-7])

while 1:
	# for i in range (numJoints):
	# 	jointInfo = p.getJointInfo(sawyerId, i)
	# 	qIndex = jointInfo[3]
	# 	if qIndex > -1:
	# 		joint_name = jointInfo[1]
	# 		p.addUserDebugParameter(joint_name.decode('utf-8'),-2.9842,2.9842,jointPoses[qIndex-7])
	# 		p.resetJointState(sawyerId,i,jointPoses[qIndex-7])
	a = 1

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
