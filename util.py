import numpy as np
import pybullet as p
import time
import math
from datetime import datetime

def get_joint_info(id,n_joints):
    # Get joints information
    joint_dict = {}
    for i in range(n_joints):
        infoJoints = p.getJointInfo(id,i)
        # print ('\njoint information',infoJoints[0],infoJoints[1],infoJoints[2],infoJoints[3],infoJoints[12])
        if infoJoints[2]!=4:
            # jointIndex, jointLowerLimit, jointUpperLimit
            joint_dict[str(infoJoints[1].decode('utf-8'))] = [int(infoJoints[0]),infoJoints[8],infoJoints[9]]  
            # print ('\njoint information',infoJoints[0],infoJoints[1],infoJoints[2],infoJoints[3],infoJoints[12]) 
    return joint_dict

class visualization:
    def __init__(self,id):
        self.id = id
    
    def vis_static(self,t):
        start = 0
        while start <= t:
            start += 0.1
            time.sleep(0.1)
    
    def vis_local_frame(self,linkId,if_base):
        if if_base == 1:
            base_pos, base_ori = p.getBasePositionAndOrientation(self.id)
            base_pos = np.asarray(base_pos); base_ori = np.asarray(base_ori)
            base_rot = np.asarray(p.getMatrixFromQuaternion(base_ori)).reshape((3,3))
            base_trans_mat = np.concatenate((base_rot,base_pos.reshape((3,1))),axis=1)
            base_trans_mat = np.concatenate((base_trans_mat,np.array([[0,0,0,1]])),axis=0)

            vec_x = np.dot(base_trans_mat,np.array([0.5,0,0,1]))[:3]
            vec_y = np.dot(base_trans_mat,np.array([0,1,0,1]))[:3]
            vec_z = np.dot(base_trans_mat,np.array([0,0,0.5,1]))[:3]

            p.addUserDebugLine(base_pos,vec_x,lineColorRGB=(1,0,0),lineWidth=4,lifeTime=0)
            p.addUserDebugLine(base_pos,vec_y,lineColorRGB=(0,1,0),lineWidth=4,lifeTime=0)
            p.addUserDebugLine(base_pos,vec_z,lineColorRGB=(0,0,1),lineWidth=4,lifeTime=0)

        for i in range(len(linkId)):
            base_pos=p.getLinkState(self.id,linkId[i])[0]; base_ori=p.getLinkState(self.id,linkId[i])[1] 
            base_pos = np.asarray(base_pos); base_ori = np.asarray(base_ori)
            base_rot = np.asarray(p.getMatrixFromQuaternion(base_ori)).reshape((3,3))
            base_trans_mat = np.concatenate((base_rot,base_pos.reshape((3,1))),axis=1)
            base_trans_mat = np.concatenate((base_trans_mat,np.array([[0,0,0,1]])),axis=0)

            vec_x = np.dot(base_trans_mat,np.array([0.2,0,0,1]))[:3]
            vec_y = np.dot(base_trans_mat,np.array([0,0.5,0,1]))[:3]
            vec_z = np.dot(base_trans_mat,np.array([0,0,0.2,1]))[:3]

            p.addUserDebugLine(base_pos,vec_x,lineColorRGB=(1,0,0),lineWidth=2,lifeTime=0)
            p.addUserDebugLine(base_pos,vec_y,lineColorRGB=(0,1,0),lineWidth=2,lifeTime=0)
            p.addUserDebugLine(base_pos,vec_z,lineColorRGB=(0,0,1),lineWidth=2,lifeTime=0)        

    def vis_one_pos(self,n_joints,joint_angle):
        joint_dict = get_joint_info(self.id,n_joints)
        count = 0
        # print ('\n sorted joint values',sorted(joint_dict.values()))
        for info in sorted(joint_dict.values()):
            p.resetJointState(self.id,info[0],joint_angle[count])
            count += 1     
    
    def add_debug_param(self,n_joints):
        ##########Important to remember: addUserDebugParameter return a value for readUserDebugParameter;
        # but the value is not equal to joint id.
        joint_dict = get_joint_info(self.id,n_joints)
        debug_dict = {}
        for j_name in joint_dict.keys():
            debug_id = p.addUserDebugParameter(j_name,joint_dict[j_name][1],joint_dict[j_name][2],0)
            debug_dict[j_name] = debug_id

        while(1):
            for j_name in joint_dict.keys():
                p.resetJointState(self.id,joint_dict[j_name][0],p.readUserDebugParameter(debug_dict[j_name]))

class get_ik:
    def __init__(self,id,rula_cvae):
        self.id = id
        self.rula_cvae = rula_cvae
    
    def rula_calc(self,jointPoses):
        jointPoses = np.array(jointPoses)/np.pi*180
        TR=jointPoses[0]; NE=jointPoses[1] 
        UA=max(jointPoses[3],jointPoses[6]) # may be modified later
        LA=max(jointPoses[4],jointPoses[7]); LL=max(jointPoses[9],jointPoses[11])
        UL=max(jointPoses[8],jointPoses[10])
        # Continuous RULA score (can be changed to other RULA later!!!)
        rula_score = np.piecewise(LA,[0<=LA<60,60<=LA<80,80<=LA<100,LA>=100],[2.,-1./20.*LA+5,1./20.*LA-3,2.]) \
                + np.piecewise(UA,[-25<=UA<0,0<=UA<20,20<=UA<50,50<=UA<100,UA>=100],[-1./25.*UA+1,1./20.*UA+1, \
                                    1./30.*UA+4./3.,1./50.*UA+2.,4.]) \
                + np.piecewise(NE,[-10<=NE<0,0<=NE<10,10<=NE<25,NE>=25],[-0.3*NE+1,0.1*NE+1.,1./15.*NE+4./3.,3.]) \
                + np.piecewise(TR,[0<=TR<10,10<=TR<25,25<=NE<50,NE>=50],[0.1*TR+1,1./15.*TR+4./3.,1./25.*TR+2.,4.]) \
                + np.piecewise(LL,[0<=LL<30,LL>=30],[1.,4.]) \
                + np.piecewise(UL,[0<=UL<20,UL>=20],[1.,4.])    
        
        print ('\n joint angles\n TR',TR,'\nUA',UA,'\nLA',LA,'\nNE',NE,'\nLL',LL,'\nUL',UL)
        # print('\n rula score',rula_score)
        return rula_score

    def analyze_jointDict(self,joint_dict):
        sort = sorted(joint_dict.values())
        ll = np.array(*[sort])[:,1]
        ul = np.array(*[sort])[:,2]
        jr = ul - ll
        return list(ll), list(ul), list(jr)

    def accurateIK(self,eefId,targetPosition,joint_dict,curr_pos,restpos,useNullSpace=False,maxIter=4,eps=1e-2):
        """
        Parameters
        ----------
        bodyId : int
        endEffectorId : int
        targetPosition : [float, float, float]
        lowerLimits : [float] 
        upperLimits : [float] 
        jointRanges : [float] 
        restPoses : [float]
        useNullSpace : bool
        maxIter : int
        threshold : float

        Returns
        -------
        jointPoses : [float] * numDofs
        """
        lowerLimits,upperLimits,jointRanges = self.analyze_jointDict(joint_dict)
        restPoses = restpos
        closeEnough = False
        iter = 0
        while (not closeEnough and iter<maxIter):
            if useNullSpace:
                jointPoses = p.calculateInverseKinematics2(self.id,eefId,targetPosition,
                    lowerLimits=lowerLimits, upperLimits=upperLimits, jointRanges=jointRanges, 
                    restPoses=restPoses,currentPositions=curr_pos)
            else:
                jointPoses = p.calculateInverseKinematics2(self.id, eefId,targetPosition,currentPositions=curr_pos)

            rula_ik = self.rula_calc(jointPoses)
            closeEnough = abs(self.rula_cvae-rula_ik) < eps
            iter=iter+1
        print("iter=",iter)
        print ('\n ik rula',rula_ik)
        return jointPoses    