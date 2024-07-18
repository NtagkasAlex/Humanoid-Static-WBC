import time
import numpy as np
import pinocchio as pin
from trajectories import * 
from robot import *
HEIGHT=0.47
OFFSET1=0.04
OFFSET2=-0.05
OFFSET_X=0.0
LEG_LIFT=0.1
LEAN_ANGLE=0.
START='Start'
SL='ShiftCOMToLeftLeg'
MR='MoveRightLeg'
SR='ShiftCOMToRightLeg'
ML='MoveLeftLeg'


class HumanoidRobotFSM:
    def __init__(self,steps,model,data,robot):
        self.state = 'Start'
        self.state_durations = {
            'Start': 0.2,  
            'ShiftCOMToLeftLeg': 0.1,
            'MoveRightLeg': 0.2,  
            'ShiftCOMToRightLeg':0.1,
            'MoveLeftLeg': 0.2,
            'Balance':2.
        }
        
        self.start_time = 0.
        self.action=self.start
        self.steps=steps
        self.next_step_indx=2
        self.foot_traj=None
        self.com_traj=None
        self.model=model
        self.data=data
        self.robot=robot

    def start(self,time):
        target_frames = ["CoM",POSTURE,"r_foot","l_foot"]
        target_weights =[2,2,2,2]

        target_types=["translation","orientation","both","both"]
        target_frames_ids = [self.model.getFrameId(f) for f in target_frames]
        target_poses = [self.data.oMf[target_frames_ids[i]].copy() for i in range(len(target_frames_ids))]


        #####COM#####
        target_com=left_foot_pos(self.model,self.data)+right_foot_pos(self.model,self.data)
        target_poses[0].translation[0]=target_com[0]/2+OFFSET_X
        target_poses[0].translation[1]=target_com[1]/2
        target_poses[0].translation[2]=HEIGHT
        
        target_poses[1].rotation= pin.rpy.rpyToMatrix(0.,LEAN_ANGLE,0.)@self.robot.start_poses[POSTURE].rotation        


        return target_frames,target_weights,target_types,target_poses
    
    def shift_com_to_leg(self,time):
        time=time-self.start_time

        target_frames = ["CoM",POSTURE,"r_foot","l_foot"]
        target_weights =[2,5,1,1]
        target_types=["translation","orientation","both","both"]
        target_frames_ids = [self.model.getFrameId(f) for f in target_frames]
        target_poses = [self.data.oMf[target_frames_ids[i]].copy() for i in range(len(target_frames_ids))]
        #####COM#####        
        target_poses[0].translation=get_com_trajectory_point(time,*self.com_traj)
        target_poses[1].rotation= pin.rpy.rpyToMatrix(0.,0.1,0.)@self.robot.start_poses[POSTURE].rotation
        target_poses[2].rotation=self.robot.start_poses["r_foot"].rotation
        target_poses[3].rotation=self.robot.start_poses["l_foot"].rotation
        ######FEEET#####
        # target_poses[2].translation[2]=0.
        # target_poses[2].translation[1]=0.
        # target_poses[2].rotation[1]=0.
        # target_poses[3].translation[2]=0.
        # target_poses[3].translation[1]=0.

        return target_frames,target_weights,target_types,target_poses
        
        ######FEEET#####
        # target_poses[2].translation[2]=0.
        # target_poses[2].translation[1]=0.

        # target_poses[3].translation[2]=0.
        # target_poses[3].translation[1]=0.

        return target_frames,target_weights,target_types,target_poses

    def move_right_leg(self,time):
        time=time-self.start_time
        target_frames = ["CoM",POSTURE,"r_foot","l_foot"]
        target_weights =[3,5,3,1]
        target_types=["translation","orientation","both","both"]
        target_frames_ids = [self.model.getFrameId(f) for f in target_frames]
        target_poses = [self.data.oMf[target_frames_ids[i]].copy() for i in range(len(target_frames_ids))]
        ######COM#####
        T=self.state_durations['MoveRightLeg']

        if time>=T:target_poses[0].translation=get_com_trajectory_point(time,*self.com_traj)
            
        else:target_poses[0].translation=get_com_trajectory_point(time,*self.com_traj)

        ##########FOOT###########
        T_SSP = T 

        if time >=T_SSP:   
            point = get_trajectory_point(T_SSP, *self.foot_traj)
        else:
            point = get_trajectory_point(time, *self.foot_traj)
        # print(point)
        
        target_poses[2].translation=point
        target_poses[2].rotation=self.robot.start_poses["r_foot"].rotation
        target_poses[3].rotation=self.robot.start_poses["l_foot"].rotation
        target_poses[1].rotation= pin.rpy.rpyToMatrix(0.,LEAN_ANGLE,0.)@self.robot.start_poses[POSTURE].rotation

        # target_poses[2].translation[0]=0.1
        # target_poses[2].translation[2]=0.2
        return target_frames,target_weights,target_types,target_poses
    def move_left_leg(self,time):
        time=time-self.start_time
        # print(time)
        target_frames = ["CoM",POSTURE,"l_foot","r_foot"]
        target_weights =[3,5,1,3]
        target_types=["translation","orientation","both","both"]
        
        target_frames_ids = [self.model.getFrameId(f) for f in target_frames]
        target_poses = [self.data.oMf[target_frames_ids[i]].copy() for i in range(len(target_frames_ids))]
        ######COM#####
        T=self.state_durations['MoveLeftLeg']

        if time>=T:target_poses[0].translation=get_com_trajectory_point(time,*self.com_traj)
            
        else:target_poses[0].translation=get_com_trajectory_point(time,*self.com_traj)

        ##########FOOT###########
        T_SSP = T  

        if time >=T_SSP:   
            point = get_trajectory_point(T_SSP, *self.foot_traj)
        else:
            point = get_trajectory_point(time, *self.foot_traj)
        # print(point)
        
        target_poses[2].translation=np.array([point[0],point[1],point[2]])
        target_poses[1].rotation= pin.rpy.rpyToMatrix(0.,LEAN_ANGLE,0.)@self.robot.start_poses[POSTURE].rotation
        target_poses[2].rotation=self.robot.start_poses["r_foot"].rotation
        target_poses[3].rotation=self.robot.start_poses["l_foot"].rotation
        # target_poses[2].translation[0]=0.1
        # target_poses[2].translation[2]=0.2
        return target_frames,target_weights,target_types,target_poses


    def transition(self,time):
        current_time = time
        elapsed_time = current_time - self.start_time
        
        if elapsed_time >= self.state_durations[self.state]:
            if self.state == START:
                self.state = SL
                target_com=left_foot_pos(self.model,self.data)
                self.com_traj=com_trajectory(self.robot.get_pos("CoM")[:2],target_com[:2]+np.array([OFFSET_X,OFFSET2]),HEIGHT,
                                             np.zeros(2),np.zeros(2),self.state_durations[SL])
                self.action=self.shift_com_to_leg
            elif self.state == SL:
                self.state = MR
                

                T=self.state_durations[MR]
                
                
                target_step=self.steps[self.next_step_indx]
                # target_com=np.array([0.12,-0.06])
                # target_com=right_foot_pos(self.model,self.data)
                self.com_traj=com_trajectory(get_com(self.model,self.data),target_step+np.array([0.,-OFFSET2]),HEIGHT,np.zeros(2),np.zeros(2),T)

                #########FOOT###########
                T_SSP = T  
                P0 = right_foot_pos(self.model,self.data) 
                P1 = np.array([target_step[0],target_step[1],P0[2]])  
                midpoint_height = LEG_LIFT
                self.foot_traj = plan_foot_trajectory(T_SSP, P0, P1, midpoint_height)
                self.next_step_indx+=1
                self.action=self.move_right_leg
            elif self.state == MR:
                self.state=SR 
                target_com=right_foot_pos(self.model,self.data)
                # target_com=np.array([0.,0.06])
                self.com_traj=com_trajectory(self.robot.get_pos("CoM")[:2],target_com[:2]+np.array([OFFSET_X,-OFFSET2]),HEIGHT,
                                             np.zeros(2),np.zeros(2),self.state_durations[SL])
                self.action=self.shift_com_to_leg

            elif self.state == SR:
                self.state=ML
                T=self.state_durations[ML]

                target_step=self.steps[self.next_step_indx]
                print(target_step)
                # target_com=np.array([0.2,0.])
                self.com_traj=com_trajectory(get_com(self.model,self.data),target_step+np.array([0.,OFFSET2]),HEIGHT,np.zeros(2),np.zeros(2),T)

                #########FOOT###########
                T_SSP = T  
                P0 = left_foot_pos(self.model,self.data) 
                P1 = np.array([target_step[0],target_step[1],P0[2]])  
                midpoint_height =LEG_LIFT 
                self.foot_traj = plan_foot_trajectory(T_SSP, P0, P1, midpoint_height)
                self.action=self.move_left_leg

                self.next_step_indx+=1

            elif self.state == ML:
                self.state = SL
                target_com=left_foot_pos(self.model,self.data)
                self.com_traj=com_trajectory(self.robot.get_pos("CoM")[:2],target_com[:2]+np.array([OFFSET_X,OFFSET2]),HEIGHT,
                                             np.zeros(2),np.zeros(2),self.state_durations[SL])
                self.action=self.shift_com_to_leg 
            if self.next_step_indx>=len(self.steps):
                #No more steps 
                self.state="Balance"
            
                target_com=left_foot_pos(self.model,self.data)/2+right_foot_pos(self.model,self.data)/2
                self.com_traj=com_trajectory(self.robot.get_pos("CoM")[:2],target_com[:2]+np.array([0.,0.]),HEIGHT,
                                             np.zeros(2),np.zeros(2),self.state_durations["Balance"])
                self.action=self.shift_com_to_leg

            self.start_time = current_time
            


    def run(self, time):
        return self.action(time)
    

def left_foot_pos(model,data):
    foot_l_id=model.getFrameId("l_foot")

    T= data.oMf[foot_l_id].copy() 
    return T.translation
def right_foot_pos(model,data):
    foot_r_id=model.getFrameId("r_foot")
    T= data.oMf[foot_r_id].copy() 
    
    return T.translation

def target_com_balance(model,data,offset=0.):
        foot_l_id=model.getFrameId("l_foot")
        foot_r_id=model.getFrameId("r_foot")

        foot_l_pose=data.oMf[foot_l_id]
        foot_r_pose=data.oMf[foot_r_id]

        return (foot_l_pose.translation[:2]+foot_r_pose.translation[:2])/2+np.array([0.,offset])
def  get_com(model,data):
    target_frames="CoM"
    target_frames_ids =model.getFrameId(target_frames) 
    T= data.oMf[target_frames_ids].copy() 
    return T.translation

