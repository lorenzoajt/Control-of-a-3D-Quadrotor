# Control-of-a-3D-Quadrotor

## Body rate and roll/pitch control (scenario 2)

Your controller will need to stabilize the rotational motion and bring the vehicle back to level attitude.

1.- Implement body rate control
- implement the code in the function GenerateMotorCommands()
  <TODO> set evidence in code
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

3.- Position/velocity and yaw angle control (scenario 3)

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

3.- Non-idealities and robustness (scenario 4)




