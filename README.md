# Control-of-a-3D-Quadrotor

## Body rate and roll/pitch control (scenario 2)

Your controller will need to stabilize the rotational motion and bring the vehicle back to level attitude.

1.- Implement body rate control
- implement the code in the function GenerateMotorCommands()
  <TODO> set evidence in code

```c++
VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
    float l = L / sqrt(2.f);
    float p_bar = momentCmd.x / l;
    float q_bar = momentCmd.y / l;
    float r_bar = - momentCmd.z / kappa;
    float c_bar = collThrustCmd;

    cmd.desiredThrustsN[0] = (p_bar + q_bar + r_bar + c_bar) / 4.f; // front left
    cmd.desiredThrustsN[1] = (-p_bar + q_bar - r_bar + c_bar) / 4.f; // front right
    cmd.desiredThrustsN[2] = (p_bar - q_bar - r_bar + c_bar) / 4.f; // rear left
    cmd.desiredThrustsN[3] = (-p_bar - q_bar + r_bar + c_bar) / 4.f; // rear right

    return cmd;
}

```
  
  
- implement the code in the function BodyRateControl()
  <TODO> set evidence in code
 
 ```c++
  V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  
    V3F error = pqrCmd - pqr;
    V3F Inertia;
    Inertia.x = Ixx;
    Inertia.y = Iyy;
    Inertia.z = Izz;

    momentCmd = Inertia * kpPQR * error;

  return momentCmd;
}
  ```
  
 
- Tune kpPQR in QuadControlParams.txt to get the vehicle to stop spinning quickly but not overshoot
    <TODO> set evidence in code
    
    ```txt
    # Angle rate gains
    kpPQR = 70, 70, 5
    
    ```

**Rotation of the vehicle about roll (omega.x) get controlled to 0 while other rates remain zero.**
<TODO> set evidence in image
  
If you come back to this step after the next step, you can try tuning just the body rate omega (without the outside angle controller) by setting QuadControlParams.kpBank = 0.


2.- Implement roll / pitch control We won't be worrying about yaw just yet.

- implement the code in the function RollPitchControl()

```c++
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

    float acc = -collThrustCmd / mass;
    float b_x_c = accelCmd.x / acc;
    float b_y_c = accelCmd.y / acc;

    float b_x = R(0,2);
    float b_x_err = b_x_c - b_x;

    float b_y = R(1, 2);
    float b_y_err = b_y_c - b_y;

    float b_x_c_dot = kpBank * b_x_err;
    float b_y_c_dot = kpBank * b_y_err;

    // following this tutorial https://www.mathsisfun.com/algebra/matrix-multiplying.html
    pqrCmd.x = (R(1,0) * b_x_c_dot - R(0,0) * b_y_c_dot) / R(2,2);
    pqrCmd.y = R(1,1) * b_x_c_dot - R(0,1) * b_y_c_dot / R(2,2);
    pqrCmd.z = 0;

  return pqrCmd;
}
```

- Tune kpBank in QuadControlParams.txt to minimize settling time but avoid too much overshoot
    ```txt
    # Angle rate gains
    kpPQR = 70, 70, 5
    
    ```

**If successful you should now see the quad level itself (as shown below), though it’ll still be flying away slowly since we’re not controlling velocity/position! You should also see the vehicle angle (Roll) get controlled to 0.**

## Position/velocity and yaw angle control (scenario 3)

Next, you will implement the position, altitude and yaw control for your quad. For the simulation, you will use Scenario 3. This will create 2 identical quads, one offset from its target point (but initialized with yaw = 0) and second offset from target point but yaw = 45 degrees.

- implement the code in the function LateralPositionControl()

```c++
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  V3F accelCmd = accelCmdFF;

    velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
    velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);

    V3F XY_dot_dot = kpPosXY * (posCmd - pos) + kpVelXY * (velCmd - vel);

    XY_dot_dot.z = 0;

    accelCmd += XY_dot_dot;
    accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
    accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
    
  return accelCmd;
}

```

- implement the code in the function AltitudeControl()
```c++
float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;
    
    float e = posZCmd - posZ;
    float e_dot= velZCmd - velZ;
    float b_z = R(2,2);
    integratedAltitudeError += e * dt;

    float u_bar = kpPosZ * e + kpVelZ * e_dot + KiPosZ * integratedAltitudeError;
    u_bar = CONSTRAIN(u_bar, -maxAscentRate / dt, maxDescentRate / dt);
    float c = (u_bar - float(CONST_GRAVITY)) / b_z;
    thrust = -mass * c;
  
  return thrust;
}

```

- tune parameters kpPosZ and kpPosZ
- tune parameters kpVelXY and kpVelZ

**If successful, the quads should be going to their destination points and tracking error should be going down (as shown below). However, one quad remains rotated in yaw.**

- implement the code in the function YawControl()
```c++
float QuadControl::YawControl(float yawCmd, float yaw)
{

    float err = yawCmd - yaw;
    yawRateCmd = kpYaw * (err);

  return yawRateCmd;

}

```

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
























