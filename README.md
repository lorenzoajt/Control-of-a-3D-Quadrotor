# Control-of-a-3D-Quadrotor

## Body rate and roll/pitch control (scenario 2)

Your controller will need to stabilize the rotational motion and bring the vehicle back to level attitude.

1.- Implement body rate control
- implement the code in the function GenerateMotorCommands()
  <TODO> set evidence in code
  '''cpp
    float l = L / sqrt(2.f); 
    float p_bar = momentCmd.x / l;
    float q_bar = momentCmd.y / l;
    float r_bar = - momentCmd.z / kappa;
    float c_bar = collThrustCmd;

    cmd.desiredThrustsN[0] = (p_bar + q_bar + r_bar + c_bar) / 4.f; // front left
    cmd.desiredThrustsN[1] = (-p_bar + q_bar - r_bar + c_bar) / 4.f; // front right
    cmd.desiredThrustsN[2] = (p_bar - q_bar - r_bar + c_bar) / 4.f; // rear left
    cmd.desiredThrustsN[3] = (-p_bar - q_bar + r_bar + c_bar) / 4.f; // rear right
  /////////////////////////////// END STUDENT CODE ////////////////////////////
    return cmd;
  '''
  
  
- implement the code in the function BodyRateControl()
  <TODO> set evidence in code
- Tune kpPQR in QuadControlParams.txt to get the vehicle to stop spinning quickly but not overshoot
    <TODO> set evidence in code

**Rotation of the vehicle about roll (omega.x) get controlled to 0 while other rates remain zero.**
<TODO> set evidence in image
  
If you come back to this step after the next step, you can try tuning just the body rate omega (without the outside angle controller) by setting QuadControlParams.kpBank = 0.


2.- Implement roll / pitch control We won't be worrying about yaw just yet.

- implement the code in the function RollPitchControl()
- Tune kpBank in QuadControlParams.txt to minimize settling time but avoid too much overshoot

**If successful you should now see the quad level itself (as shown below), though it’ll still be flying away slowly since we’re not controlling velocity/position! You should also see the vehicle angle (Roll) get controlled to 0.**

## Position/velocity and yaw angle control (scenario 3)

Next, you will implement the position, altitude and yaw control for your quad. For the simulation, you will use Scenario 3. This will create 2 identical quads, one offset from its target point (but initialized with yaw = 0) and second offset from target point but yaw = 45 degrees.

- implement the code in the function LateralPositionControl()
- implement the code in the function AltitudeControl()
- tune parameters kpPosZ and kpPosZ
- tune parameters kpVelXY and kpVelZ

**If successful, the quads should be going to their destination points and tracking error should be going down (as shown below). However, one quad remains rotated in yaw.**

- implement the code in the function YawControl()
- tune parameters kpYaw and the 3rd (z) component of kpPQR

Tune position control for settling time. Don’t try to tune yaw control too tightly, as yaw control requires a lot of control authority from a quadcopter and can really affect other degrees of freedom. This is why you often see quadcopters with tilted motors, better yaw authority!

**Hint:** For a second order system, such as the one for this quadcopter, the velocity gain (kpVelXY and kpVelZ) should be at least ~3-4 times greater than the respective position gain (kpPosXY and kpPosZ).

## Non-idealities and robustness (scenario 4)
In this part, we will explore some of the non-idealities and robustness of a controller. For this simulation, we will use Scenario 4. This is a configuration with 3 quads that are all are trying to move one meter forward. However, this time, these quads are all a bit different:

- The green quad has its center of mass shifted back
- The orange vehicle is an ideal quad
- The red vehicle is heavier than usual
1.- Run your controller & parameter set from Step 3. Do all the quads seem to be moving OK? If not, try to tweak the controller parameters to work for all 3 (tip: relax the controller).

2.- Edit AltitudeControl() to add basic integral control to help with the different-mass vehicle.

3.- Tune the integral control, and other control parameters until all the quads successfully move properly. Your drones' motion should look like this:

## Tracking trajectories

Now that we have all the working parts of a controller, you will put it all together and test it's performance once again on a trajectory. For this simulation, you will use Scenario 5. This scenario has two quadcopters:

- the orange one is following traj/FigureEight.txt
- the other one is following traj/FigureEightFF.txt - for now this is the same trajectory. For those interested in seeing how you might be able to improve the performance of your drone by adjusting how the trajectory is defined, check out Extra Challenge 1 below!
How well is your drone able to follow the trajectory? It is able to hold to the path fairly well?

## Performance Metrics
The specific performance metrics are as follows:

- scenario 2

  - roll should less than 0.025 radian of nominal for 0.75 seconds (3/4 of the duration of the loop)
roll rate should less than 2.5 radian/sec for 0.75 seconds

- scenario 3

  - X position of both drones should be within 0.1 meters of the target for at least 1.25 seconds
Quad2 yaw should be within 0.1 of the target for at least 1 second

- scenario 4

  - position error for all 3 quads should be less than 0.1 meters for at least 1.5 seconds

- scenario 5

  - position error of the quad should be less than 0.25 meters for at least 3 seconds
























