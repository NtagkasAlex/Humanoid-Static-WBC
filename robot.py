import pinocchio as pin

import numpy as np
import copy
import proxsuite
POSTURE="chest"
class Robot():
    def __init__(self,q0) -> None:
        load_model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf("./robot_dart/utheque/icub/icub.urdf", \
                                                                       package_dirs="./robot_dart/utheque/icub", \
                                                                       root_joint=pin.JointModelFreeFlyer())
        self.model =load_model
        self.data = self.model.createData()
        self.q=self.dart_to_pin(q0)
    def default_frames(self):
        model=copy.copy(self.model)
        data=copy.copy(self.data)
        target_frames=["CoM",POSTURE,"r_foot","l_foot"]
        target_frames_ids = [model.getFrameId(f) for f in target_frames]
        poses = [data.oMf[target_frames_ids[i]].copy() for i in range(len(target_frames_ids))]
        self.start_poses = {target_frames[i]:poses[i] for i in range(len(target_frames))}

    def init_frame(self):  
        self.contact_frames=["fr_left_foot","br_left_foot","fl_left_foot","bl_left_foot","fr_right_foot","br_right_foot","fl_right_foot","bl_right_foot"]
        self.contact_frames_ids = [self.model.getFrameId(f) for f in self.contact_frames]
        
        self.v=np.zeros(self.model.nv)

        # Compute the center of mass position
        pin.centerOfMass(self.model, self.data, self.q, self.v)

        com_world = self.data.com[0]#WOrld Frame CoM
        root_joint_id = self.model.getJointId('root_joint')
        root_joint_placement = self.data.oMi[root_joint_id]  

        # Transform the CoM position to the local frame of the root_joint
        com_local = root_joint_placement.actInv(com_world)


        # Now define the CoM frame
        self.com_frame_name = "CoM"
        self.com_frame = pin.Frame(
            self.com_frame_name,
            self.model.getFrameId('root_joint'),
            self.model.getJointId('root_joint'),
            pin.SE3(np.eye(3), com_local), 
            pin.FrameType.OP_FRAME
        )

        # Add the frame to the model
        self.model.addFrame(self.com_frame)
        self.data = self.model.createData()



    def print(self,link_name):
        id=self.model.getFrameId(link_name)
        T=self.data.oMf[id]
        print(T)

    def get_pos(self,name):
        target_frames=name
        target_frames_ids =self.model.getFrameId(target_frames) 
        T= self.data.oMf[target_frames_ids].copy() 
        return T.translation
    def show_contacts(self,robot):
        for frame in self.contact_frames:
            robot.set_draw_axis(frame,size=0.1)
            
    def fk_all(self, q, v = None):
        if v is not None:
            pin.forwardKinematics(self.model, self.data, q, v) # FK and Forward Velocities
        else:
            pin.forwardKinematics(self.model, self.data, q) # FK
        pin.updateFramePlacements(self.model, self.data) # Update frames
    def dart_to_pin(self,q):
        q_pin=pin.neutral(self.model)
        # print(q_pin)
        q_pin[:3]=q[3:6]

        q_pin[3:7]=axis_angle_2_quaternion(q[:3])

        q_pin[7:]=q[6:]
        # print(q_pin)
        return q_pin
    def dart_to_pin_vel(self,vel):
        v_pin=np.zeros(self.model.nv)
        # print(q_pin)
        v_pin[:3]=vel[3:6]

        v_pin[3:6]=vel[:3]

        v_pin[6:]=vel[6:]
        # print(q_pin)
        return v_pin
   

def decode_qp_sol(model,sol):
    dq = sol[:model.nv]
    tau = sol[model.nv:2*model.nv]
    F_left = sol[2*model.nv:2*model.nv + 4 * 3]
    F_right = sol[2*model.nv + 4 * 3:]
    return dq, tau, F_left, F_right


def axis_angle_2_quaternion(axis_angle):
    # return np.array([1,0,0,0])
    theta = np.linalg.norm(axis_angle)
    if theta < 1e-6:
        return np.array([0, 0, 0, 1])
    
    ax,ay,az = axis_angle / theta

    qw=np.cos(theta/2.)

    qx=ax*np.sin(theta/2.)

    qy=ay*np.sin(theta/2.)

    qz=az*np.sin(theta/2.)


    return np.array([qx, qy, qz,qw])
def quaternion_to_rotation_matrix(q):
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
    ])



