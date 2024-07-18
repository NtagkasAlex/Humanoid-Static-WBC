# Humanoid-Static-WBC
Humanoid walking with whole body control involves the coordinated management of all joints and limbs to achieve a stable walking motion.
* Footstep generation
* Center of Mass(CoM) trajectory Planning
* Foot trajectory planning
* Quadratic Programming (QP) Controller to track the above trajectories.
* Finite State Machine (FSM) to coordinate between the different states of
the movement.


## Finite State Machine

The FSM for a humanoid walking algorithm is one of the most important aspects. I will be using something similar to this:

1. DSP is a double support phase where both feet stay on the ground. For me
this will be shifting the CoM to the opposite side of the next step’s side.

2. SSP is the single support phase where we must lift the respective leg and
move it to the next footstep position , retaining the robot’s balance in the
process.
<p>
    <img src="images/image copy.png" alt="Description of image" width="400" >
</p>

## Cubic Hermite Splines Trajectories
Cubic Hermite splines are a type of spline where each piece is a third-degree
polynomial specified in Hermite form. The spline ensures continuity of both the
function and its first derivative.

## CoM Trajectory
On SSP, as we see below, we have to move the CoM towards the landing
stop of the opposite foot.

On DSP the CoM has to remain on the offset point and wait for the next
step.

<p>
    <img src="images/image copy 3.png" alt="Description of image" width="400" >
</p>

## Foot Trajectory

This is an inline equation: $ p(\frac{T_{SSP}}{2}) = \begin{bmatrix}
        \frac{p_{0x}+p_{1x}}{2} \\ \frac{p_{0y}+p_{1y}}{2}\\ H\\
    \end{bmatrix} $.
