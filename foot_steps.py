import numpy as  np
import RobotDART as rd
def generate_footsteps(distance, step_length, foot_spread):
    contacts = []
    array=[]
    def append_contact(x, y):
        contacts.append(create_step(x,y))
        array.append(np.array([x,y]))
    append_contact(0., +foot_spread)
    append_contact(0., -foot_spread)
    x = 0.
    y = foot_spread
    while x < distance:
       if distance - x <= step_length:
           x += min(distance - x, 0.5 * step_length)
       else:  # still some way to go
           x += step_length
       y = -y
       append_contact(x, y)
    append_contact(x, -y)  # now x == distance
    return contacts,array

def create_step(x,y):
    pose = np.random.random([6])
    pose=np.array([0,0,0,x+0.03,y,0.001])

    size=np.array([0.15,0.1,0.001])
    
    return rd.Robot.create_box(size, pose, "free", 1., [1, 0, 0, 1.])

def draw_steps(simulator,steps_list):
    for step in steps_list:
        step.fix_to_world()
        simulator.add_visual_robot(step)
