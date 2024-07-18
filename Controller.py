import pinocchio as pin

import numpy as np

import proxsuite

class QP():
    def __init__(self,model,data,foot):
        N_foot = 2
        self.contact_frames=["fr_left_foot","br_left_foot","fl_left_foot","bl_left_foot","fr_right_foot","br_right_foot","fl_right_foot","bl_right_foot"]

        if foot=="left":
            N_foot=1
            self.contact_frames=["fr_left_foot","br_left_foot","fl_left_foot","bl_left_foot"]
        elif foot=="right":
            N_foot=1
            self.contact_frames=["fr_right_foot","br_right_foot","fl_right_foot","bl_right_foot"]

        self.N_contacts = N_foot * 4 # 4 contact points per foot
        self.qp_dim = model.nv + model.nv + 3 * self.N_contacts
        self.qp_dim_eq = model.nv # dynamics!
        self.qp_dim_in = 5 * self.N_contacts # Friction cones constraints! ProxSuite supports double limits: d_min <= Cx <= d_max!
        self.qp = proxsuite.proxqp.dense.QP(self.qp_dim, self.qp_dim_eq, self.qp_dim_in)        


        self.contact_frames_ids = [model.getFrameId(f) for f in self.contact_frames]


        self.qp_init = False

        self.prev_sol = np.zeros((self.qp_dim,))


   
    def update_targets(self,model,data,target_frames,weights,types,poses):

        self.target_frames = target_frames
        self.target_weights = weights
        self.target_types=types
        self.target_frames_ids = [model.getFrameId(f) for f in self.target_frames]
        self.target_poses = poses
        
       
    def qp_inverse_dynamics(self,model,data,q,v ,mu = 0.5, n = np.array([0., 0., 1.]), t1 = np.array([1., 0., 0.]), t2 = np.array([0., 1., 0.])):

    
        contact_frames=self.contact_frames
        contact_frames_ids=self.contact_frames_ids
        target_weights=self.target_weights
        target_frames_ids=self.target_frames_ids
        target_poses=self.target_poses

        fk_all(model,data,q, v)
        # Then compute all Jacobians needed
        pin.computeJointJacobians(model, data, q)
        # We also need the time derivative of the Jacobians
        pin.computeJointJacobiansTimeVariation(model, data, q, v)
        # Mass Matrix
        pin.crba(model, data, q)
        # Coriolis/Gravity forces
        pin.nonLinearEffects(model, data, q, v)

        # Let's create the A matrix
        A = np.zeros((model.nv, self.qp_dim))
        # put the mass matrix in the correct position
        A[:model.nv, :model.nv] = np.copy(data.M)
        # selection matrix
        S = np.eye(model.nv)
        S[:6, :6] = 0. # we cannot control the base!
        A[:, model.nv:2*model.nv] = -S
        # contact jacobians
        # for frame, frame_id in zip(contact_frames, contact_frames_ids):
        for i in range(len(contact_frames)):
            frame_id = contact_frames_ids[i]
            J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            A[:, 2*model.nv + i * 3 : 2*model.nv + (i + 1) * 3] = -J[:3, :].T
        # Let's create the b vector
        b = -np.copy(data.nle)

        # Inequality Constraints (Friction cones)
        C = np.zeros((5 * len(contact_frames), self.qp_dim))
        dmin = np.zeros((5 * len(contact_frames),))
        dmax = np.zeros((5 * len(contact_frames),))
        for i in range(len(contact_frames)):
            # dot(f, n) > 0
            eps=1e-6
            C[i * 5, 2*model.nv + i * 3 : 2*model.nv + (i + 1) * 3] = n # normal vector (world frame)
            dmin[i * 5] = eps
            dmax[i * 5] = None # no upper limit
            # -μ dot(f, n) <= dot(f, t1) <= μ dot(f, n)
            C[i * 5 + 1, 2*model.nv + i * 3 : 2*model.nv + (i + 1) * 3] = t1 - mu * n
            dmin[i * 5 + 1] = None
            dmax[i * 5 + 1] = -eps
            C[i * 5 + 2, 2*model.nv + i * 3 : 2*model.nv + (i + 1) * 3] = t1 + mu * n
            dmin[i * 5 + 2] = eps
            dmax[i * 5 + 2] = None
            # -μ dot(f, n) <= dot(f, t2) <= μ dot(f, n)
            C[i * 5 + 3, 2*model.nv + i * 3 : 2*model.nv + (i + 1) * 3] = t2 - mu * n
            dmin[i * 5 + 3] = None
            dmax[i * 5 + 3] = -eps
            C[i * 5 + 4, 2*model.nv + i * 3 : 2*model.nv + (i + 1) * 3] = t2 + mu * n
            dmin[i * 5 + 4] = eps
            dmax[i * 5 + 4] = None

        num_target_tasks = sum([6 if t == 'both' else 3 for t in self.target_types])
        W = np.zeros((num_target_tasks + len(contact_frames) * 6, self.qp_dim))
        t = np.zeros((num_target_tasks + len(contact_frames) * 6,))

        # Contact frames tasks
        for i in range(len(contact_frames)):
            weight = 10. * np.max(target_weights)
            frame_id = contact_frames_ids[i]
            J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            dJ = pin.getFrameJacobianTimeVariation(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            W[i * 6 : (i + 1) * 6, : model.nv] = J * weight  # update W
            # We do not use a PD-controller here because it can generate jitter!
            t[i * 6 : (i + 1) * 6] = (-dJ @ v) * weight  # update t

        # Target frames tasks
        W_idx = len(contact_frames) * 6
        t_idx = len(contact_frames) * 6
        for i in range(len(target_weights)):
            weight = target_weights[i]
            frame_id = target_frames_ids[i]
            T_wd = target_poses[i]
            task_type = self.target_types[i]
            
            # get Jacobian and time derivative of Jacobian
            J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            dJ = pin.getFrameJacobianTimeVariation(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            
            # Compute current pose and velocity
            current_pose = data.oMf[frame_id].copy()
            current_vel = pin.getFrameVelocity(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).vector
            Kp = 500.
            Kd = 2. * np.sqrt(Kp)
            if task_type == 'translation':
                # Translation-only task
                error = (T_wd.translation - current_pose.translation).reshape(-1)
                J_translation = J[:3, :]  # Use only the translational part of the Jacobian
                dJ_translation = dJ[:3, :]
               
                p_ddot = Kp * error - Kd * current_vel[:3]
                W[W_idx : W_idx + 3, : model.nv] = J_translation * weight  # update W
                t[t_idx : t_idx + 3] = (p_ddot - dJ_translation @ v) * weight  # update t
                W_idx += 3
                t_idx += 3

            elif task_type == 'orientation':
                # Orientation-only task
                error = pin.log3(T_wd.rotation@current_pose.rotation.T )
                # error = pin.log3(current_pose.rotation.T@T_wd.rotation )
                J_orientation = J[3:, :] 
                dJ_orientation = dJ[3:, :]
                # Kp = 10.
                # Kd = 2. * np.sqrt(Kp)
                p_ddot = Kp * error - Kd * current_vel[3:]
                W[W_idx : W_idx + 3, : model.nv] = J_orientation * weight  # update W
                t[t_idx : t_idx + 3] = (p_ddot - dJ_orientation @ v) * weight  # update t
                W_idx += 3
                t_idx += 3

            elif task_type == 'both':
                # Both translation and orientation task
                error_translation = (T_wd.translation - current_pose.translation).reshape(-1)
                # error_orientation = pin.log3(current_pose.rotation.T@T_wd.rotation )
                error_orientation = pin.log3(T_wd.rotation@current_pose.rotation.T )

                error = np.hstack((error_translation, error_orientation))
               
                p_ddot = Kp * error - Kd * current_vel
                W[W_idx : W_idx + 6, : model.nv] = J * weight  # update W
                t[t_idx : t_idx + 6] = (p_ddot - dJ @ v) * weight  # update t
                W_idx += 6
                t_idx += 6

        # Let's create the big Q matrix and q vector
        # regularization tasks (very important)
        reg = np.eye(self.qp_dim) * 1e-6
        reg[:model.nv,:model.nv] = np.eye(model.nv) * 1e-3# We want to minimize accelerations as much as possible
        hands_ids=[22,23,24,25,26,27,28,32,33,34,35,36,37,38]
        for idx in hands_ids:
            reg[idx, idx] *= 100
            # reg[idx+model.nv, idx+model.nv] *= 10 
        Q = W.T @ W + reg
        q_o = -W.T @ t

        # Let's solve the QP
        if not self.qp_init: # in first iteration we initialize the model
            self.qp.init(Q, q_o, A, b, C, dmin, dmax)
        else: # otherwise, we update the model
            self.qp.update(Q, q_o, A, b, C, dmin, dmax)
        # Let's solve the QP
        self.qp.solve(self.prev_sol, None, None)

        self.qp_init = True

        self.prev_sol = np.copy(self.qp.results.x)

        return np.copy(self.qp.results.x)
    
def fk_all(model,data, q, v = None):
    if v is not None:
        pin.forwardKinematics(model, data, q, v) # FK and Forward Velocities
    else:
        pin.forwardKinematics(model, data, q) # FK
    pin.updateFramePlacements(model, data) # Update frames