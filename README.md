# AE 544 Analytical Dynamics Programming Project 02
### By: Jasmine Nakladov

## Introduction
This project asks to write three (3) numerical programs to simulate three scenarios, all of which will include the approach or hit of a singularity or ambiguity. The scenarios are as follows:

1. Singularity for Asymmetric Euler Angle Sets
2. Ambiguity of Euler Parameters / Quaternions
3. Classical Rodriques Parameters

This will be accomplished by integrating an attitude or dynamics equation with initial conditions. The angular velocity vector initial condition will be manipulated to ensure the singularity/ambiguity is approached. An additional goal of this project is to investigate whether different numerical integrators affect the singularity problem in different ways. The investigated integrators will be as follows: *ode45* for a nonstiff example, and *ode15s* for a stiff example.

Finally, a 3D animation will be included to aid in analyzing each simulation.

Any references used in this project are located at the bottom of this .md file. Matlab toolboxes used for this code include the Aerospace Toolbox as well as the Robotics System Toolbox.

## Singularity for Asymmetric Euler Angle Sets
The first scenario that will be examined is the singularity of Asymmetric Euler Angles. The specific sequence chosen was the classic 3-2-1 Yaw Pitch Roll rotation. For this rotation, the geometric singularity occurs at a pitch angle of +/- $90^\circ$, where the two remaining angles are not uniquely defined. The following process will be followed for Scenarios 2. and 3. as well. 

The following equations were used for this simulation, gathered from the textbook's **Appendix B.2 Mapping Between Body Angular Velocity Vector and the Euler
Angle Rates**:

$$
\dot{\theta} = [B(\theta)] \omega
$$

$$
[B(\theta)] = \frac{1}{cos{\theta_2}}
\begin{bmatrix}
0 & sin{\theta_3} & cos{\theta_3} \\
0 & cos{\theta_2} cos{\theta_3} & -cos{\theta_2} sin{\theta_3} \\
cos{\theta_2} & sin{\theta_2} sin{\theta_3} & sin{\theta_2} cos{\theta_3}
\end{bmatrix}
$$

In the matrix equation, it can be clearly seen that a singularity occurs at $\theta_2 = 90^\circ$, as this would cause the $\frac{1}{cos{\theta_2}}$ term to be divided by 0. A function was created to solve the $\dot{\theta} = [B(\theta)] \omega$ equation, as seen in the attached *.m* code. In this function, each Euler angle $\theta$ is extracted from the integrator's input state vector, with ode45 being the first integrator tested. The inputs to the function are the Euler angles and the body's angular velocity. The function then defines the above $[B(\theta)]$ matrix and $\dot{\theta}$ equation, which will use the integrator's updated values of the angular rates as inputs.

Above the function, the body angular velocity vector, initial angles, timespan, and the ode's input state vector are defined. The angular velocity vector, defined as *omega_body*, is important and serves as the main way to manipulate the angles towards singularity. The actual values of the $\omega$ vector are arbitrary, and many variations were tested to see which would bring the pitch angle, $\theta_2$, close to the singularity. The final values chosen for this part of the project are as follows:

$$
[\omega] = 
\begin{bmatrix}
0.1 \\
0.2 \\
0.1
\end{bmatrix}
 rad/s
$$

The initial Euler angles were all set to 0 rad, and the timespan was set to 100 seconds. For the integrator, the input state vector contains $\theta$ and time provided by the function at each integration step. The *ode45* function then repeatedly calls on the predefined *angular_rates* function, the timespan, and the initial angular conditions, outputting $\dot{\theta}$ and integrating for the Euler angles. Finally, in the code, $\theta$ is converted to degrees for an easier visual and plotted alongside the time *t*. The output can be seen below:

<p align="center">
  <img src="https://github.com/user-attachments/assets/c446f309-f566-4a05-8376-0ae87a3ef221" alt="Singularity for Asymmetric Euler Angles Figure Using ode45" width="500">
</p>

With this code, the pitch angle approaches $-90^\circ$, reaching up to $-89.8484^\circ$. When the pitch angle approaches this value in the figure at around 18, 45, 70, and 95 seconds, gimbal lock occurs the remaining two angles, yaw and roll, react accordingly. Both angles experience discontinuities and/or sharp jumps at the same time as gimbal lock occurs. This is because when this happens, the system loses one of its degrees of freedom, causing the yaw and roll angles to become unstable. 

The same process was then repeated with the ode15s function, used to solve stiff problems as opposed to ode45's nonstiff problems. The only difference between this and the previously described code is the integrator. Below are the results:

<p align="center">
  <img src="https://github.com/user-attachments/assets/fdc93cdb-94ef-409c-aa40-8e34202ba24d" alt="Singularity for Asymmetric Euler Angles Figure Using ode15s" width="500">
</p>

As can be seen in the plots, there are some important differences between the two integrators. The Pitch angle appears to be the same for both integrators, oscillating every ~25 seconds. The Yaw angle still shows discontinuities at $-90^\circ$ when the singularity is hit, but they are more subtle with the *ode45* integrator. It is the opposite for the Roll angle, where the discontinuities are large with *ode45* but more subtle with *ode15s*. The *ode15s* function overall seems more stable, dampening the singularity behavior. 

## Ambiguity of Euler Parameters / Quaternions
The second scenario that will be examined is the ambiguity of the Euler Parameters. Euler Parameters (or Quaternions) do not have a singularity causing gimbal lock due to their nature, as a division by 0 never occurs. However, the parameters have their own ambiguity in that a positive or negative vector both represent the same rotation. In the code, the quaternions were calculated with the same body angular velocity conditions and similar initial conditions, with a 1 being included for the fourth parameter to ensure that the norm is 1. The calculation of the quaternions is structured the same way as that of the Euler angles, with one difference being that the values are normalized after integrating, once again to ensure that the norm is 1. The function differs as well, but only in the equations it represents. Equations 3.105 and 3.106 from the textbook were used to calculate the parameters and can be seen as follows.

$$
\dot{\beta} = \frac{1}{2} * [B(\beta)]^\beta \omega
$$

$$
\begin{pmatrix}
\dot{\beta}_0 \\
\dot{\beta}_1 \\
\dot{\beta}_2 \\
\dot{\beta}_3 \\
\end{pmatrix} = \frac{1}{2} *
\begin{bmatrix}
\beta_0 & -\beta_1 & -\beta_2  & -\beta_3 \\
\beta_1 & \beta_0 & -\beta_3  & \beta_2 \\
\beta_2 & \beta_3 & \beta_0  & -\beta_1 \\
\beta_3 & -\beta_2 & \beta_1  & \beta_0  
\end{bmatrix}
\begin{pmatrix}
0 \\
\omega_1 \\
\omega_2 \\
\omega_3 \\
\end{pmatrix}
$$

The process was repeated for both positive and negative parameters. Below is the resulting plot after ode45 was used, showing both the positive and negative parameters. 

<p align="center">
  <img src="https://github.com/user-attachments/assets/bbbf16cd-df11-4746-9bad-6eb534045c87" alt="Euler Parameters Over Time Using *ode45*" width="500">
</p>

It can be seen in this plot that the positve and negative quaternions are mirrors of each other. The same process can once again be done with *ode15s* in order to see how the parameters behave with a different integrator. These results can be seen below:

<p align="center">
  <img src="https://github.com/user-attachments/assets/22ad8621-119b-41bf-aeab-a6215fc26002" alt="Euler Parameters Over Time Using *ode15s*" width="500">
</p>

As can be seen, there is no difference between the results of the integrators. 

To further show that the quaternions represent the same rotation no matter their sign, the Euler parameters were converted back into Euler angles. This conversion shows that the angles are the same, no matter what the sign is. A graphical representation of this can be seen below.

<p align="center">
  <img src="https://github.com/user-attachments/assets/84922ad4-e794-47d5-af45-4785eefd2585" alt="Euler Parameters Converted to Euler Angles" width="500">
</p>

This representation confirms that quaternions represent the same rotation, no matter their sign.

## Classical Rodriques Parameters
The last scenario that will be examined is the CLassical Rodriques Parameters singularity. For this representation, a singularity occurs at $ **\phi** = +/- 180 ^\circ$, with **\phi** representing the principal angle . The same process as the previous two simulations was followed for this one, with a function being created to represent the appropriate equations and integrators being used to derive the parameters. The initial angular velocity 

The equations used for this function are as follows, noted as equations 3.130 and 3.131 in the textbook.

$$
[\dot{q}] = \frac{1}{2} * [[I] + [\bar{q}] + qq^T]^\beta \omega
$$

$$ 
[\dot{q}] = \frac{1}{2} *
\begin{bmatrix}
1 + q_1^2 & q_1q_2 - q_3 & q_1q_3 + q_2 \\
q_2q_1 + q_3 & 1 + q_2^2 & q_2q_3 - q_1 \\
q_3q_1 - q_2 & q_3q_2 + q_1 & 1 + q_3^2
\end{bmatrix} ^\beta
\begin{pmatrix}
\omega_1 \\
\omega_2 \\
\omega_3 \\
\end{pmatrix}
$$

The resulting graph, solved with the *ode45* function, is as follows:

<p align="center">
  <img src="https://github.com/user-attachments/assets/6c3a8728-6d87-480e-acde-23c6b4393077" alt="Classical Rodriques Parameters Using *ode45*" width="500">
</p>

As can be seen, there is a large discontinuity at around 13 seconds. Although the graph appears to be only at 0 prior to the discontinuity, when examining the values in the Matlab workspace this is untrue. The values start at the initial condition of [0 0 0], and they increase as they reach the discontinuity. This discontinuity occurs because the principal angle hits $180 ^\circ$. When integrated with *ode15s*, the same result can be seen:

<p align="center">
  <img src="https://github.com/user-attachments/assets/034599ae-26a0-49c5-89b9-fdfd992b7d9e" alt="Classical Rodriques Parameters Using *ode15s*" width="500">
</p>

To prove that the discontinuity occurs at $180 ^\circ$, the principal angle was calculated and plotted. The equation used to calculate the angle is equation 3.116 in the textbook:

$$
q = tan \frac{\phi}{2}\hat{e}
$$

which can then be rearranged for \phi:

$$
\phi = 2* tan^{-1}(q)
$$

In the code, the Rodriques Parameters were normalized and then plugged into the \phi equation. The resulting plot is as follows:

<p align="center">
  <img src="https://github.com/user-attachments/assets/2176c180-8b4d-45d3-b346-a882728b803b" alt="Principal Angle for CLassical Rodrigues Parameters" width="500">
</p>

As can be seen in this plot, the principal angle reaches $180 ^\circ$ at the same time as the discontinuity occurs.

## 3D Animations
3D animations were created to further analyze the scenarios. The animations were created with the use help of Matlab's Help Center, Matlab's file exchange, and ChatGPT. The animation is set up in the following way:
1. An object, specifically a cube, is created for investigation and is later patched together.
2. A figure and axis that will depict the animation are created using the *quiver3* and *text* functions.
3. The *hgtransform* function is used to ensure that the animation is smooth.
4. The body axis of the cube is created and labeled.
5. A GIF file name is created.

The animation is then created in a for loop as follows:

1. The Euler angles from the integrator are separated into yaw, pitch, and roll.
2. The Euler angles are converted into rotation matrices, found in the Textbook as equations 3.32a, 3.32b, and 3.33c. These can be seen below:

$$ 
[M_1(\theta)] = 
\begin{bmatrix}
1 & 0 & 0 \\
0 & cos\theta & sin\theta \\
0 & -sin\theta & cos\theta
\end{bmatrix}
$$

$$ 
[M_2(\theta)] = 
\begin{bmatrix}
cos\theta & 0 & -sin\theta \\
0 & 1 & 0 \\
sin\theta & 0 & cos\theta
\end{bmatrix}
$$

$$ 
[M_3(\theta)] = 
\begin{bmatrix}
cos\theta & sin\theta & 0 \\
-sin\theta & cos\theta & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

3. The transformation is then set for the cube and the body fixed axis and its labels are manually updated.
4. Finally, the GIF is created using a *gif* function found in Matlab's file exchange. 

Below is the animation depicting the Euler Angle rotation. This file is also located in the submitted folder and can be opened with a browser.

<p align="center">
  <img src="https://github.com/user-attachments/assets/cdd76a0c-24bc-4a81-9161-30227e4550e2" alt="Euler Angles 3D Animation" width="500">
</p>

As can be seen, the animation is smooth but shows instability when the pitch angle reaches $90 ^\circ$ and the remaining two axes align. This is consistent with the plot discussed in the earlier section.

For the Euler parameter, the same steps were followed, with the rotation matrix being equation 3.93 in the textbook:

$$
[C] = 
\begin{bmatrix}
\beta_0^2 + \beta_1^2 - \beta_2^2 - \beta_3^2 & 2(\beta_1 \beta_2 + \beta_0 \beta_3) & 2(\beta_1 \beta_3 - \beta_0 \beta_2) \\
2(\beta_1 \beta_2 - \beta_0 \beta_3) & \beta_0^2 - \beta_1^2 + \beta_2^2 - \beta_3^2 & 2(\beta_2 \beta_3 + \beta_0 \beta_1) \\
2(\beta_1 \beta_3 + \beta_0 \beta_2) & 2(\beta_2 \beta_3 - \beta_0 \beta_1) & \beta_0^2 - \beta_1^2 - \beta_2^2 + \beta_3^2
\end{bmatrix}
$$

An additional difference in the steps is that the rotation matrix had to be edited into a 3x3 matrix as that is the form that the *hgtransform* expects. The resulting GIF can be seen here:

<p align="center">
  <img src="https://github.com/user-attachments/assets/63dcc696-c8e8-4339-a298-c119bda7e9a4" alt="Euler Parameters 3D Animation" width="500">
</p>

As can be seen, the rotation is smooth and does not have any discontinuities due to the nature of quaternions.

## Conclusion
All three scenarios, Euler Angle singularity, Euler Parameter ambiguity, and Classical Rodrigues Parameter singularity, were successfully modeled in order to investigate their effect on an object's attitude. The Euler Angle singularity, occurring at +/- $90 ^\circ$, was approached as close as $-89.8484^\circ$. Gimbal lock occurred and the other two axes reacted accordingly, becoming unstable as can be seen in both the plots and the animation. The Euler Parameter ambiguity, that being that both a positive and a negative quaternion represent the same rotation, was modeled accordingly. The positive and negative quaternions were mirrors of each other. When converted into Euler angles, it could be seen that both the positive and negative parameters did indeed lead to the same rotation. Finally, the Classical Rodrigues Parameter singularity occurs at a principal angle of +/- $180 ^\circ$, and was represented with the created code and plot the same as the other scenarios. The plot shows the angles increasing slightly before reaching the singularity, at which the integrator stops as the simulation "blows up". To verify the results, the principal angle was plotted, and it can be seen that the discontinuity occurred at the same time as when the angle reached $180 ^\circ$.

## References
1. Programming Project 01 Project Instructions
2. AE544 Class Notes, Specifically Chapter 3
3. Analytical Mechanics of Space Systems, 2nd edition, by Hanspeter Schaub and John L. Junkins 
4. Matlab Help Center
   1. Matlab ode45: https://www.mathworks.com/help/matlab/ref/ode45.html
   2.  Matlab Choose an ode Solver: https://www.mathworks.com/help/matlab/math/choose-an-ode-solver.html
   3. Matlab quat3dcm: https://www.mathworks.com/help/aerotbx/ug/quat2dcm.html
   4. Matlab quat2eul: https://www.mathworks.com/help/robotics/ref/quat2eul.html
6. ChatGPT was consulted for help with MathJax syntax, as well as inserting figures into the .md file and creating 3D animations
7. 3D Animation resources to support ChatGpt's information about how to create an animation as well as to export the animation as a GIF
   1. Animating an Objects Trajectory in MATLAB with HGtransform: https://www.mathworks.com/videos/animating-an-objects-trajectory-in-matlab-with-hgtransform-97224.html
   2. To make the animation smooth, hgtransform: https://www.mathworks.com/help/matlab/ref/hgtransform.html
   3. To create axis, quiver3: https://www.mathworks.com/help/matlab/ref/quiver3.html
   4. For creating axis labels, set: https://www.mathworks.com/help/matlab/ref/set.html
   5. For combining vertices, patch: https://www.mathworks.com/help/matlab/ref/patch.html
   6. File Exchange gif function: https://www.mathworks.com/matlabcentral/fileexchange/63239-gif
  
