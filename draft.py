import numpy as np
#################test list and array
# a = np.array([1,2])
# print ('\n-a',-a)

# #################test dict values
# a = {'a':[1,2],'b':[2,3]}
# c = np.array([*a.values()])
# print ('\n vlue',c.shape,type(c))
# t = np.array(a.values())
# print ('\nt',t,t.shape)

# ##################test tuple
# a = tuple([1,2,3])
# print ('\n a1',a[1])

# joint information 0 b'jL5S1_rotx' 4 -1 b'L5_f1'
# joint information 1 b'jL5S1_roty' 0 7 b'L5'
# joint information 2 b'jL4L3_rotx' 4 -1 b'L3_f1'
# joint information 3 b'jL4L3_roty' 4 -1 b'L3'
# joint information 4 b'jL1T12_rotx' 4 -1 b'T12_f1'
# joint information 5 b'jL1T12_roty' 4 -1 b'T12'
# joint information 6 b'jT9T8_rotx' 4 -1 b'T8_f1'
# joint information 7 b'jT9T8_roty' 4 -1 b'T8_f2'
# joint information 8 b'jT9T8_rotz' 4 -1 b'T8'
# joint information 9 b'jT1C7_rotx' 4 -1 b'Neck_f1'
# joint information 10 b'jT1C7_roty' 0 8 b'Neck_f2'
# joint information 11 b'jT1C7_rotz' 4 -1 b'Neck'
# joint information 12 b'jC1Head_rotx' 4 -1 b'Head_f1'
# joint information 13 b'jC1Head_roty' 4 -1 b'Head'
# joint information 14 b'jRightC7Shoulder_rotx' 4 -1 b'RightShoulder'
# joint information 15 b'jRightShoulder_rotx' 0 9 b'RightUpperArm_f1'
# joint information 16 b'jRightShoulder_roty' 4 -1 b'RightUpperArm_f2'
# joint information 17 b'jRightShoulder_rotz' 0 10 b'RightUpperArm'
# joint information 18 b'jRightElbow_roty' 4 -1 b'RightForeArm_f1'
# joint information 19 b'jRightElbow_rotz' 0 11 b'RightForeArm'
# joint information 20 b'jRightWrist_rotx' 4 -1 b'RightHand_f1'
# joint information 21 b'jRightWrist_rotz' 4 -1 b'RightHand'
# joint information 22 b'jRightHandCOM' 4 -1 b'RightHandCOM'
# joint information 23 b'jLeftC7Shoulder_rotx' 4 -1 b'LeftShoulder'
# joint information 24 b'jLeftShoulder_rotx' 0 12 b'LeftUpperArm_f1'
# joint information 25 b'jLeftShoulder_roty' 4 -1 b'LeftUpperArm_f2'
# joint information 26 b'jLeftShoulder_rotz' 0 13 b'LeftUpperArm'
# joint information 27 b'jLeftElbow_roty' 4 -1 b'LeftForeArm_f1'
# joint information 28 b'jLeftElbow_rotz' 0 14 b'LeftForeArm'
# joint information 29 b'jLeftWrist_rotx' 4 -1 b'LeftHand_f1'
# joint information 30 b'jLeftWrist_rotz' 4 -1 b'LeftHand'
# joint information 31 b'jLeftHandCOM' 4 -1 b'LeftHandCOM'
# joint information 32 b'jRightHip_rotx' 0 15 b'RightUpperLeg_f1'
# joint information 33 b'jRightHip_roty' 0 16 b'RightUpperLeg_f2'
# joint information 34 b'jRightHip_rotz' 4 -1 b'RightUpperLeg'
# joint information 35 b'jRightKnee_roty' 0 17 b'RightLowerLeg_f1'
# joint information 36 b'jRightKnee_rotz' 4 -1 b'RightLowerLeg'
# joint information 37 b'jRightAnkle_rotx' 4 -1 b'RightFoot_f1'
# joint information 38 b'jRightAnkle_roty' 4 -1 b'RightFoot_f2'
# joint information 39 b'jRightAnkle_rotz' 4 -1 b'RightFoot'
# joint information 40 b'jRightBallFoot_roty' 4 -1 b'RightToe'
# joint information 41 b'jLeftHip_rotx' 0 18 b'LeftUpperLeg_f1'
# joint information 42 b'jLeftHip_roty' 0 19 b'LeftUpperLeg_f2'
# joint information 43 b'jLeftHip_rotz' 4 -1 b'LeftUpperLeg'
# joint information 44 b'jLeftKnee_roty' 0 20 b'LeftLowerLeg_f1'
# joint information 45 b'jLeftKnee_rotz' 4 -1 b'LeftLowerLeg'
# joint information 46 b'jLeftAnkle_rotx' 4 -1 b'LeftFoot_f1'
# joint information 47 b'jLeftAnkle_roty' 4 -1 b'LeftFoot_f2'
# joint information 48 b'jLeftAnkle_rotz' 4 -1 b'LeftFoot'
# joint information 49 b'jLeftBallFoot_roty' 4 -1 b'LeftToe'

a = np.array([1,2])
b = np.array([2,3])
print('\n a-b',a-b)