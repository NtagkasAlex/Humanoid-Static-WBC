## Humanoid-Static-WBC
Humanoid walking with whole body control involves the coordinated management of all joints and limbs to achieve a stable walking motion.
* Footstep generation
* Center of Mass(CoM) trajectory Planning
* Foot trajectory planning
* Quadratic Programming (QP) Controller to track the above trajectories.
* Finite State Machine (FSM) to coordinate between the different states of
the movement.


# Finite State Machine

The FSM for a humanoid walking algorithm is one of the most important aspects. I will be using something similar to this:

1. DSP is a double support phase where both feet stay on the ground. For me
this will be shifting the CoM to the opposite side of the next step’s side.

2. SSP is the single support phase where we must lift the respective leg and
move it to the next footstep position , retaining the robot’s balance in the
process.
<p>This is a paragraph that contains an image.
    <img src="images/image copy.png" alt="Description of image" width="500" height="600">
</p>
