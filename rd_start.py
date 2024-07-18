import RobotDART as rd
import numpy as np
import pinocchio as pin
from robot import *
from foot_steps import *
import time
from Controller import QP
from motion import HumanoidRobotFSM,right_foot_pos,left_foot_pos,get_com
import matplotlib.pyplot as plt

your_model_packages = [("icub_description", "./robot_dart/utheque/icub/icub_description")]
robot = rd.Robot("./robot_dart/utheque/icub/icub.urdf", your_model_packages)
robot.set_actuator_types("torque")
simu = rd.RobotDARTSimu(0.001)
simu.set_collision_detector("bullet")

robot.set_base_pose(np.array([0,0,0,0,0,0.48]))

simu.add_checkerboard_floor()
simu.add_robot(robot)

robot.set_color_mode("material")

g_config = rd.gui.GraphicsConfiguration()
g_config.shadow_map_size = 256
g_config.shadowed = False
g_config.transparent_shadows = False
graphics = rd.gui.Graphics(g_config)

simu.set_graphics(graphics)

dt = simu.timestep()
steps_sim,steps=generate_footsteps(0.5,0.08,0.06)
draw_steps(simu,steps_sim)


icub=Robot(robot.positions())

icub.init_frame()
icub.fk_all(icub.q)
icub.default_frames()

icub.show_contacts(robot)
model=icub.model
data=icub.data

# robot.set_draw_axis("neck_1")
robot.set_draw_axis("chest")
# robot.set_draw_axis("torso_1")
# robot.set_draw_axis("head")

controller=QP(model,data,"both")
controllerR=QP(model,data,"right")
controllerL=QP(model,data,"left")

q=icub.q
v=icub.v


com=[]
right_leg_plot=[]
left_leg_plot=[]
start = time.time()

fsm=HumanoidRobotFSM(steps,model,data,icub)

while (simu.scheduler().next_time() < 4. and  not simu.graphics().done()):

    timer=simu.scheduler().next_time()
    
    fsm.transition(timer)

    if fsm.state=='MoveRightLeg':
        controllerL.update_targets(model,data,*fsm.run(timer))
        sol=controllerL.qp_inverse_dynamics(model,data,icub.q,icub.v)

    elif fsm.state=='MoveLeftLeg':
        controllerR.update_targets(model,data,*fsm.run(timer))
        sol=controllerR.qp_inverse_dynamics(model,data,icub.q,icub.v)

    else:
        controller.update_targets(model,data,*fsm.run(timer))
        sol=controller.qp_inverse_dynamics(model,data,icub.q,icub.v)

    dq, tau, F_left, F_right = decode_qp_sol(model,sol)
    if np.all(tau == 0.):
        print("Failed",timer)
    hands_ids=[22,23,24,25,26,27,28,32,33,34,35,36,37,38]
    modified_list = [x - 1 for x in hands_ids]

    tau[modified_list]=0.

    robot.set_commands(tau)
   
    simu.step_world()
    #for plotting
    com.append(get_com(model,data))
    right_leg_plot.append(right_foot_pos(model,data))
    left_leg_plot.append(left_foot_pos(model,data))
 
    v_next=robot.velocities()
    q_next=robot.positions()

    icub.q=icub.dart_to_pin(q_next)
    icub.v=icub.dart_to_pin_vel(v_next)


plot=False
if plot:
    data = np.array(com)
    # Create the plots
    fig, axs = plt.subplots(1, 3, figsize=(15, 6))

    axs[0].plot(data[:, 0])
    axs[0].set_title(f'Axis x')

    axs[1].plot(data[:,1])
    axs[1].set_title(f'Axis y')

    axs[2].plot(data[:, 2])
    axs[2].set_title(f'Axis z')

    plt.tight_layout()
    plt.show()

    data = np.array(right_leg_plot)

    # Create the plots
    fig, axs = plt.subplots(1, 3, figsize=(15, 6))
    axs[0].plot(data[:, 0])
    axs[0].set_title(f'Axis x')

    axs[1].plot(data[:,1])
    axs[1].set_title(f'Axis y')

    axs[2].plot(data[:, 2])
    axs[2].set_title(f'Axis z')

    plt.tight_layout()
    plt.show()

    data = np.array(left_leg_plot)

    fig, axs = plt.subplots(1, 3, figsize=(15, 6))

    axs[0].plot(data[:, 0])
    axs[0].set_title(f'Axis x')

    axs[1].plot(data[:,1])
    axs[1].set_title(f'Axis y')

    axs[2].plot(data[:, 2])
    axs[2].set_title(f'Axis z')

    plt.tight_layout()
    plt.show()
