# AE 544 Analytical Dynamics Programming Project 02
### By: Jasmine Nakladov

## Introduction
This project asks to explore the control performance of a three-link system with torque applied to each link (shown below) by utilizing the system's Hamiltonian as the Lyapunov function of a feedback control system. This will be accomplished by examining the problem in three perspectives:

1. Global stability for all initial conditions
2. Robustness to certain modem errors
3. Freedom of control law design

<p align="center">
  <img src="https://github.com/user-attachments/assets/19003448-d175-40e6-9b70-a45c7aa21dbe" alt="Three Link Diagram" width="400">
  <br>
  <em>Figure 1. Three-link System</em>
</p>

A 3D animation / GIF will be included to aid in analyzing the situation. Example 8.9 in the Analytical Mechanics of Space Systems textbook will be used as guidance for this project. Any other references used in this project are located at the bottom of this .md file. Matlab toolboxes used for this code include the Parallel Computing Tololbox, and the Symbolic toolbox. No other instructions are needed to run this code and replicate the results.

Please note: The code may initially take a few minutes to run due to the parallel computing. Additionally, a pop up may ask you to "overwrite" the GIF as it alredy exists due to the nature of the code. One can choose to cancel or overwrite for the same results.

## Example 8.9 Summary

Example 8.9 of the textbook examines the same three link manipulator as shown above, with the goal of bringing the manipulator to rest from the motion created by the torque. The generalized cooridnates chosen are as follows:

$$
q =
\begin{bmatrix}
\theta_1\\
\theta_2\\
\theta_3
\end{bmatrix}
$$

The following equations of motion (seen in the Project Instructions as well as the example) can be used to describe the system.

$$
[M]\ddot{q} + [\dot{M}]\dot{q} - \frac{1}{2} \dot{q}^T[M_q]\dot{q} = Q
$$

Below is the system mass matrix, also given in the Project Instructions as well as the textbook example. It is derived in "Part 1" of this project, attached as a hand derivation and confirmed by Matlab.

$$
[M(q)] =
\begin{bmatrix}
(m_1 + m_2 + m_3)l_1^2 & (m_2 + m_3)l_1l_2cos(\theta_2 - \theta_1) & m_3l_1l_3cos(\theta_3 - \theta_1)\\
(m_2 + m_3)l_1l_2cos(\theta_2 - \theta_1) & (m_2 + m_3)l_2^2 & m_3l_2l_3cos(\theta_3-\theta_2) \\
m_3l_1l_3cos(\theta_3 - \theta_1) & m_3l_2l_3cos(\theta_3-\theta_2) & m_3l_3^2
\end{bmatrix}
$$

Notable assumptions of the example, which hold for this project as well, are that 1. the Lagrangian, and by extension the Hamiltonian and Lyapunov, is chosen to be the same as the kinetic energy. The velocity feedback control laws are then defined as seen here, with P1 and P2 being positive scalar gains:

$$
Q_1 = -P_1\dot{q} 
$$
$$
Q_2 = -P_2[M(q)]\dot{q} 
$$

A simulation is made to examine the control performance. The following table shows the initial parameters, which will be used for this project as well. These initial conditions are depicting the movement of the third link only, with links 1 and 2 being at rest (after the control is implemented, as the three links affect each other). In this scenario, $ Q_1 $ stabilizes all three links at once, allowing the kinetic energy of the third link to affect the other 2. $ Q_2 $ stabilizes the third link separately.

<p align="center">
<br>
  <em>Table 1: Initial Parameters of Isolated Motion</em>

| Parameter                | Value              | Units      |
|--------------------------|--------------------|------------|
| $l_i$                   | 1                  | m          |
| $m_i$                   | 1.0                | kg         |
| $P_1$                   | 1.0                | $kg*m^2/s$    |
| $P_2$                   | 0.72               | $kg*m^2/s$    |
| $\mathbf{x}(t_0)$       | [90, 30, 0]        | deg        |
| $\dot{\mathbf{x}}(t_0)$ | [0.0, 0.0, 10]     | deg/s      |
</p>

The following plots are then derived and will be replicated in this project.

<p align="center">
  <img src="https://github.com/user-attachments/assets/c4902f97-4ae6-4b69-8f6c-519f35125a8c" alt="Example 8.9 Plots" width="550">
    <br>
  <em>Figure 2. Example 8.9 Plots</em>
</p>

## M Matrix Confirmation
The first part of the code is to confirm the given M matrix and serves as a supplement to the "Part 1 Derivation" document which derives the M matrix by hand. 

The hand calculation is redone in the code by defining variables using the *syms* function and defining the positions of each link based on the given diagram. The generalized coordinates are chosen to be $\theta_1 , \theta_2$, and $\theta_3$. The *jacobian* function is then used to compute the derivatives (velocities) of each position vector. Finally, the *jacobian* function is used once more to take partial derivatives of the kinetic energy in order to obtain generalized momentum and later mass for the final mass matrix.

## Global Stability Investigation
Global stability was investigated for each control with fixed gains by simulating 100 Monte-Carlo simulations, which use repeated random sampling to model and analyze the probability of different outcomes. 

The gains are defined to be the same as in Example 8.9 (1 $kg*m^2/s$ for P1 and 0.72 $kg*m^2/s$ for P2). The mass matrix is defined once again, this time as a function as it is expected to change and having it as a function allows it to be evaluated at any time in the simulation.

For the Monte-Carlo simulations, a range of $[-\pi, \pi]$ is chosen for the joint angles. A range of $[-1, 1]$ is chosen for the joint velocities. The conditions are sufficient to evaluate the *global* stability of the system. The Monte-Carlo simulation is created with the use of the *parfor* function. The simulation simulates both control laws for varying initial conditions that fit the described range while checking if the system converges to rest (with a threshold of <0.1).

A function was created for this purpose and will be used throughout the rest of the code. The function's inputs are the initial joint angles, initial joint velocities, total simulation time, time step, mass matrix defined as a function, control law defined as a function, and a definition of true or false for saving the torque data. The function's outputs are time, qout (which defines a matrix holding both position and velocity), and Qout (which defines the torque). The simulation computes a new mass matrix as well as a torque input each time step. It then solves for the joint accelearations. The function is created to simulate what happens to the three link system as time goes on based on a specified control law by simulating its dynamics over time.

The success rate of the convergence test for $Q_1$ ranges each time that the code is run from approximately 3% to 10% (Based on the random conditions). The success rate for $Q_2$ is always 100%. This implies that $Q_1$, which stabilizes all three links at once, is often not successful, unlike $Q_2$, which stabilizes the third link separately. This further implies that $Q_1$'s method of stabilizing the system introduces complexity that hinders the convergence, while $Q_2$'s method is more robust. Below is a diagram of the success rate, as well as a table.

<p align="center">
<br>
  <em>Table 2: Tabulated Success Rates</em>

| Control Law | Successful Trials | Total Trials | Success Rate |
|-------------|--------------------|--------------|--------------|
| $Q_1$        | 6                  | 100          | 6%           |
| $Q_2$        | 100                | 100          | 100%         |
</p>

These results show that $Q_1$ is only efficient for some initial conditions, not all. This is because its equation, $Q_1 = -P_1\dot{q} $ does not consider the M matrix, unlike $Q_2$. Therefore, if the initial conditions are small, the control law succeeds, but if they are large, then $Q_1$ fails to converge. Since $Q_2$ does consider the M matrix, it takes into account the nonlinear dynamics.

## Comparison of Control Laws
The next section of the project asks to compare the performance of the two control laws by showing $\dot{q}$ curves and the corresponding effect of each control law. This was done by replicating the plots in Example 8.9, seen in Figure 2.

As the goal is to replicate Example 8.9's figures, the same initial conditions were used, with only link 3 rotating while the other two links remain at zero. The previously defined function was used to gather the new time, positions, velocities, and torques. The replicated figures are seen below. When compared to Figure 2, one can see that they replicate the example's plots exactly.  

<p align="center">
  <img src="https://github.com/user-attachments/assets/05bb0664-7281-4b89-a687-56de33777496" alt="Q Dot Vector Components" width="450">
    <br>
  <em>Figure 4. q̇ Vector Components </em>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/39ee7f3a-8f4a-437d-9ea0-6d46a0c8ce29" alt="Vector Q Components" width="450">
    <br>
  <em>Figure 5. Control Vector Q Components</em>
</p>

Following the logic of the example, it can be seen that $Q_1$ is able to stabilize the system, brigning all three links to rest, but at the same time the kinetic energy of the third link is partially transmitted to links 1 and 2, affecting the entire system. This is clearly seen in Figure 6, where the at-rest link is controlled by $Q_1$ to move from rest before converging to zero again. $Q_2$ on the other hand, as it only tries to stabilize link 3, ensures that links 1 and 2 remain at rest, controlling the third link separately. Therefore, $Q_2$ has better state error convergence to zero.

For further analysis, a sample of trajectories of $\dot{q}_1$ were plotted as seen below.

<p align="center">
  <img src="https://github.com/user-attachments/assets/362c995c-86f3-4d07-852c-f4e49ffd3c8c" alt="Trajectories" width="450">
    <br>
  <em>Figure 6. q̇1 Trajectories for Q1 and Q2 </em>
</p>

Here, the difference between the two control laws can once again be seen. $Q_1$ control shows clearly unstable trajectories, while $Q_1$ shows stability for all depicted trials. This also supports the information and success rates found with the Monte-Carlo simualtion.

## Introducing Inertia Matrix Error
Next, error was introduced to the inertia matrix in $Q_2$, introducing a new control law of:

$$
Q_3 = -P_2([M(q)] + [ΔM])\dot{q}
$$

A comparison was made between the original $Q_2$ and $Q_3$. ΔM was defined as a symmetric matrix with small values multiplied by epsilon, representing the size of the modeling error. Epsilon was chosen as small (0.01) and large (0.9) to see its effect on the control law. The matrix was confirmed to be positive, definite by checking its eigen values for negativity.

<p align="center">
  <img src="https://github.com/user-attachments/assets/25d4663f-2743-4f7d-9924-9c6df768b51b" alt="Mass Matrix Effect" width="450">
    <br>
  <em>Figure 7. Effect of Mass Matrix Error (State Magnitude vs Time)</em>
</p>

As can be seen in the figure below, a small epsilon (or, a small deviation from the M matrix) resulted in an almost identical response as $Q_2$. A large epsilon/deviation resulted in a larger deviation and difference in the $\dot{q}$ state. Either way, the system still converged to 0. This is one of the benefits of defining the Lyapunov function as the Hamiltonian. With this method, the function is independent of the system's dynamics and only depends on the torques and velocities to which forces are applied to.

## 3D Animation
3D animations were created to further analyze the scenarios. The animations were created with the use help of Matlab's Help Center, Matlab's file exchange, and ChatGPT. The animation is set up in the following way, similar to Project 1:

1. An object, specifically the three links, is created for investigation and is later patched together.
2. A figure and axis that will depict the animation are created.
3. The hgtransform function is used to ensure that the animation is smooth.
4. The dynamics of each link is defined as shown in Part 1 of the project. 
5. A GIF file name is created.

This specific animation depicts the stabilizing movement of the third link while links 1 and 2 remain at rest. This animationi corresponds to figures 4 and 5.

<p align="center">
  <img src="https://github.com/user-attachments/assets/9611d965-7a48-4adb-9611-69d6c42f9d33" alt="Three Link GIF" width="450">
    <br>
  <em>Figure 8. Three Link System GIF </em>
</p>

Using this animation for analysis, it can be seen that the $Q_2$ control law quickly and efficiently stabilized link 3 without affecting links 1 and 2.

## Conclusion
In conclusion, it was seen that $Q_2$, which controlled link 3 separately from links 1 and 2, was the superior control law due to its reliance on the inertia mass matrix. $Q_1$ was shown to only reach the convergence of 0 for small initial conditions, succeeding approxinaltey 3% to 10% of the time as opposed to $Q_2$'s 100% global stability success rate. This was supported by seeing the first state's trajectories under each control law. Thanks to the method of depicting the Lyapunov function as a Hamiltonian, the control vector $Q_3$ stabilized despite the M matrix differing from its initial values. The overall project showed the benefits of this method.

## References
1. Programming Project 02 Project Instructions
2. AE544 Class Notes, Specifically Chapter 8
3. Analytical Mechanics of Space Systems, 2nd edition, by Hanspeter Schaub and John L. Junkins 
4. Matlab Help Center
   1. Matlab ode45: https://www.mathworks.com/help/matlab/ref/ode45.html
   2.  Matlab jacobian: https://www.mathworks.com/help/symbolic/sym.jacobian.html
   3. Matlab parfor: https://www.mathworks.com/help/parallel-computing/parfor.html
   4. Monte-Carlo Simulation: https://www.mathworks.com/discovery/monte-carlo-simulation.html
   5. Matlab File Exchange, Monte-Carlo: https://www.mathworks.com/matlabcentral/fileexchange/55306-monte-carlo-estimation-examples-with-matlab
   
   ChatGPT was used to help plot the trajectories correctly, with the AI suggesting the following functions instead of subplot. These resources were used to support ChatGPT's suggestion,
   
   6. Matlab tiledlayout: https://www.mathworks.com/help/matlab/ref/tiledlayout.html
   7. Matlab nexttile: https://www.mathworks.com/help/matlab/ref/nexttile.html

5. Monte Carlos on YouTube for Path Trajectories: https://www.youtube.com/watch?v=691fsAgdmok&ab_channel=MonteCarlos
6. ChatGPT was consulted for help with MathJax syntax, as well as inserting figures and tables into the .md file and suggesting including optional torque output in the *system_simulation* function.
7. 3D Animation resources
   1. File Exchange gif function: https://www.mathworks.com/matlabcentral/fileexchange/63239-gif
  








![image](https://github.com/user-attachments/assets/19003448-d175-40e6-9b70-a45c7aa21dbe)
![image](https://github.com/user-attachments/assets/c4902f97-4ae6-4b69-8f6c-519f35125a8c)

https://www.investopedia.com/terms/m/montecarlosimulation.asp

https://www.mathworks.com/help/parallel-computing/parfor.html

![Monte Carlo Success Comparison](https://github.com/user-attachments/assets/3441c285-e981-4c5a-874a-5923ba42633a)

![Control Vector Q Components](https://github.com/user-attachments/assets/39ee7f3a-8f4a-437d-9ea0-6d46a0c8ce29)

![Q Dot Vector Components](https://github.com/user-attachments/assets/05bb0664-7281-4b89-a687-56de33777496)




![Trajectories](https://github.com/user-attachments/assets/362c995c-86f3-4d07-852c-f4e49ffd3c8c)

![Mass Matrix Effect](![Mass Matrix Effect](https://github.com/user-attachments/assets/25d4663f-2743-4f7d-9924-9c6df768b51b)
)

![ThreeLinkRobotAnimation](https://github.com/user-attachments/assets/9611d965-7a48-4adb-9611-69d6c42f9d33)
