## Project scope
Development of an adaptive controller to support package
delivery missions.

The tasks are intended to address the control system development in an incremental
manner, starting from the design model up to the testing in a representative simulation
environment.

## Project tasks
**Task 1: simulation model implementation**
- 1.1 Implement in Simulink the dynamics of a hexacopter UAV with the geometric and
inertial data in the provided MATLAB script.
The six propellers are evenly placed in the plane of the body frame according to the
following figure
- 1.2 Given the nominal model of the UAV, compute analytically the commands $\Omega_{rcmd}$, $𝑟 = 1, … , 6$ to trim the UAV at $p = [0\; 0\; 1]^T$
where 𝑝 is the inertial position.
- 1.3 Given the nominal model of the UAV, compute (analytically or numerically) the
commands $\Omega_{rcmd}$, $𝑟 = 1, … , 6$ to trim the UAV at $p = [2\; 0\; 0]^T$
where $𝑣_𝑖$ is the inertial velocity.

**Task 2: Baseline controller implementation, tuning and testing**
- 2.1 Implement in Simulink the control architecture for position-heading tracking control presented. 
- 2.2 Tune the motion controllers to have a desired behavior 
with respect to a setpoint tracking case.
To tune the gains, you must properly specify control objectives (e.g., overshoot, settling time, disturbance rejection capabilities, ...) that the controller should provide in design
conditions. Consider working on simplified/linearized models when performing the tuning and taking into
account constraints from actuator and sensor dynamics (e.g., bandwidth, saturation,
delays).
- 2.3.1  Verify the behavior for the setpoint tracking case used to tune the controller, Retune the controller if needed, a trajectory tracking scenario (specify a suitable trajectory considering speed no larger than $4-5m/s$ and a suitable metric to evaluate the tracking performance).
- 2.3.2 Simulate a reduction of the propellers effectiveness during the trajectory tracking
scenario and assess the impact on the tracking capabilities of the baseline control
system.

**Task 3: Adaptive augmentation**
- 3.1 Assume a scenario where the hexacopter UAV is used for package delivery.
Considering the package as a rigid body attached at the bottom with of the UAV at a
distance $r_c \in \mathcal{R}^3$ from the origin of the body. Specify suitable inertial properties for the package and then properly modify the UAV
simulator. Verify the behavior of the baseline control system in a representative scenario, assuming that the inertial properties of the package are completely unknown to the
baseline controller.
- 3.2 Consider the following approximation of the uncertain UAV dynamics for position
control augmentation $$m \dot{v}_i = -mg e_3 + f_e + \Lambda_f f_c $$
where $\Lambda_f$ is a diagonal control effectiveness matrix and $f_e$ represents an exogenous disturbance force. Derive the corresponding predictor dynamics and the corresponding update laws for the development of a Predictor-Based MRAC design assuming $f_e$ be constant.
<!-- Remark: You are not asked to augment the attitude baseline controller. Under which
assumptions is this reasonable? -->
- 3.3 Adaptive control law implementation and testing. Based on the uncertain dynamics derived in point 3.2, implement in Simulink the
PBMRAC controller, in augmentation to the baseline position controller developed in
Task 2. Test the control law in a representative scenario. What is the effect of the added package mass on the control effectiveness?

**Task 4: test in delivery scenario**

Consider developing a suitable delivery scenario where the hexacopter must deliver a package in an
urban like environment (consider at least three buildings with suitable dimensions).
Specify take off and landing locations and set up a mission for the UAV splitting the delivery in take
off, cruise and landing phases.
Assume the landing location to be in front of a building and consider the environment and the map to
be known.
Test the behavior of the adaptive control system in the presence of reasonable unforeseen events
(wind gusts/degradation of the actuation capabilities).

## Organization
All tasks are performed in the matlab script `MAIN.m`, which already calls the appropriate simulink files to run the simulations.
<!-- All tasks are included in a single matlab and a single simulink files, such that by running the file `MAIN_exam_t3.m` the simulink file `task3_MRAC3.slx` is already simulated.  -->

## Usage
All the documents can be run either in MATLAB v2024b or higher, and on simulink v2024b or higher.

## Results
To adress the inertial position trim problem, a leveled UAV (null pitch and roll angles) was considered, with $ R = I_3$. Between the infinite solutions (given by an underdetermined problem in 4 equations and 6 unknowns) the selected trim condition 
corresponds to equal thrust on all rotors such that:
$$\Omega_{rcmd} = \sqrt{\frac{mg}{6k_f}}$$
The velocity trim problem is treated differently in the sense that the hypothesis of leveled UAV falls, but the trim condition is chosen as for the previous problem, giving:
$$\Omega_{rcmd} = \sqrt{\frac{mgcos\theta}{6k_f}}$$

The control architectures for position and heading tracking as the allocation are implemented as explained in `P2CS1`. 
To tune the position control gains the `Control systems tuner` simulink toolbox was used. The following requirements were set:
- Loop tuning: $\omega_c=0.8\; rad/s$ `HARD`
- Step tracking: I order, $Ts=1\; s$
- Step rejection: maximum amplitude =1, maximum settling time = 1, maximum damping = 1

To define requirements on the attitude control it was first linearized and modeled as a second order system. The requirements were specified in terms of natural frequency and damping ratio, and were differentiated between the response along the z-axis and that along the x- and y-axes. As a matter of fact the different behavior observed along the z-axis indicated that a different control approach was required in that direction. In particular, a high damping ratio was selected for the z-axis in order to limit overshoot and oscillatory behavior. On the other hand, lower damping was chosen on the other axes to allow faster transient response.
z-axis requirements:
- $\omega_n = 15\;rad/s$ 
- $\xi = 0.9$ 

x and y axes requirements:
- $\omega_n = 15\;rad/s$ 
- $\xi = 0.1$ 

It is importante to notice that in both cases the control system is tuned to be sufficiently slower than the actuators, whose bandwidth is of $20 rad/s$, in order to avoid interpherence. Moreover the position control is slower than the attitude control ensuring a sufficient scale separation. 
The root mean square error on the position is used to evaluate the tracking performance.

Reduction of propellers effectiveness results on a circular trajectory (considering **no wind gust** and $1000s$ of simulation):
<!-- CIRCULAR TRAJECTORY - with delays-->
| Kind of reduction | rmse1 | rmse2 |
|--------|--------|--------|
| All propellers working properly $\Lambda = \mathbf{1}$ |0.2251 | 0.2191|
| Only 1 propellers is reduced with $\lambda_1 = 0.3$ | 0.3259  |  0.3102 |
| All propellers reduced, with $\Lambda = \mathbf{0.6}$|  0.4988| 0.4880|


Reduction of propellers effectiveness results on a circular trajectory (considering **wind gust** and $1000s$ of simulation):
<!-- CIRCULAR TRAJECTORY . with delays-->
| Kind of reduction | rmse1 | rmse2 |
|--------|--------|--------|
| All propellers working properly $\Lambda = \mathbf{1}$ | 0.2275| 0.2216|
| Only 1 propellers is reduced with $\lambda_1 = 0.3$ | 0.3168|0.3040 |
| All propellers reduced, with $\Lambda = \mathbf{0.6}$| 0.4983 | 0.4877 |

The following conditions were considered:
- the failure of a single propeller with the lowest possible value of $\lambda_1$;
- the failure of all propellers with the lowest possible value of $\Lambda$;

It was possible to observe that for $\lambda \leq 0.5$ the failure of a single propeller was possible, whereas for a higher number of "failed" propellers, the UAV was not properly controlled and its ENU position error was diverging.
The resulting reduction in tracking capability is evaluated using the root mean square error, in particular rmse1 is considering all the positions starting from $t=0\;s$ while, rmse2 instead is defined starting from $t = 10\;s$.

Then a package delivery scenario was considered and a package was defined as follows:
- $m = 0.5\;kg$ package mass
- $h = 0.03\;m$ package height
- $l = 0.2\;m$ package length
- $w = 0.1\;m$ package width
- $rc = [0\;0\;-0.1]\;m$ package position with respect to the UAV center of mass

Its static moment was computed in order to update the overall mass of the UAV (including the package), while accounting for the appropriate inertial contributions introduced by the package.

To implement the PB-MRAC the following adaptive laws were considered:
$$\dot{\hat{\mathbf{x}}}_p = \mathbf{A_p}\; \hat{\mathbf{x}}_p + \mathbf{B_p}\;(\mathbf{\Lambda_f}\mathbf{u} + \mathbf{\varphi}_p^T\hat{\mathbf{\theta}}_p)+ \mathbf{L} \hat{e}_p - g e_3$$
$$\mathbf{u}_{AD} = (\hat{\mathbf{\Lambda}}_f^{-1}-\mathbf{I})\mathbf{u}_{BL}- \hat{\mathbf{\Lambda_f}}^{-1}\varphi_p^T\hat{\theta}_p$$
$$\dot{\hat{\theta}}_a = \Gamma_a \varphi_aB_p^T\hat{e}_p$$
Where 
$$A_p = [zeros(3,3), eye(3); zeros(3,3), zeros(3,3)]$$
$$B_p = 1/M(1,1)[zeros(3,3); eye(3)] $$
$$\varphi^T=eye(3)$$
<!-- $$d=[zeros(3,1); 1/M(1,1)(fe-gM(1,1)\mathbf{e}_3)];$$ -->
where $\theta_p = f_e$, and $M$ is the mass of the UAV (without package). A wind gust is also modeled in the UAV dynamics as a step disturbance, whose intensity is chosen to study different behaviours.
Once the model was implemented in Simulink, the controller gains were tuned accordingly. $B_p$ is uncertain since dependent on the uncertain mass (package inertial properties are unknown to the baseline controller), thus some matching conditions would be necessary and $\hat{\theta}_p$, $\hat{\Lambda}_f$ will contain also uncertainty on the mass.

It is important to observe that it was necessary to saturate the integration of $\dot{\hat{\theta}}_a$, and in particular the integration for $\lambda$ which otherwise would lead to unfeasible matrix inversion. 

In particular, a comparison between the RMSE obtained from the non-augmented problem and from the PB-MRAC problem was performed, and the matrices $L$ and $\Gamma$ were selected so as to minimize the RMSE for the PB-MRAC case. The choice fell on:
- $\mathbf{L} = \mathbf{15}$
- $\mathbf{\Gamma}_a = \mathbf{0.1}$

<!-- Trajectory tracking performance for a sinusoidal trajectory (considering **no wind gust** and $1000s$ of simulation): -->
<!-- Sinusoidal trajectory - with delays -->
<!-- |Setting| rmse1| rmse2|
|--------|--------|--------|
|PB-MRAC with no package and no degradation on propellers| 0.3252 | 0.3107|
|Baseline with no package and no degradation on propellers|0.3167 | 0.3128|
|PB-MRAC with package and no degradation on propellers| 0.3516 | 0.3375|
|Baseline with package and no degradation on propellers| 0.4078 | 0.4044 | -->

Trajectory tracking performance for a sinusoidal trajectory (considering **no wind gust** and $1000s$ of simulation):
<!-- Sinusoidal trajectory - with delays -->
|Setting| rmse1| rmse2|
|--------|--------|--------|
|PB-MRAC with no package and no degradation on propellers| 0.3144 | 0.3107 |
|Baseline with no package and no degradation on propellers| 0.3167 | 0.3128 |
|PB-MRAC with package and no degradation on propellers| 0.3423 | 0.3377 |
|Baseline with package and no degradation on propellers| 0.4076 | 0.4043 |
    
Trajectory tracking performance for a sinusoidal trajectory (considering a **wind gust** and $1000s$ of simulation):
<!-- Sinusoidal trajectory - with delays -->
|Setting| rmse1| rmse2|
|--------|--------|--------|
|PB-MRAC with no package and no degradation on propellers| 0.3212 | 0.3174 |
|Baseline with no package and no degradation on propellers| 0.3188 | 0.3148 |
|PB-MRAC with package and no degradation on propellers| 0.3414 | 0.3366 |
|Baseline with package and no degradation on propellers| 0.4076 | 0.4043  |


<!-- Trajectory tracking performance for a sinusoidal trajectory (considering a **wind gust** and $1000s$ of simulation): -->
<!-- Sinusoidal trajectory - with delays -->
<!-- |Setting| rmse1| rmse2|
|--------|--------|--------|
|PB-MRAC with no package and no degradation on propellers|0.3303| 0.3158|
|Baseline with no package and no degradation on propellers| 0.3230 | 0.3191 |
|PB-MRAC with package and no degradation on propellers|0.3490 | 0.3343 |
|Baseline with package and no degradation on propellers|  0.4149 |0.4116 | -->

**NOTE: the added wind gust is always $adapt.wind=[10;10;0];$**

A proper trajectory was then defined considering the following delivery problem:
- delivery of a package starting from Pansushi to building B12 in Politecnico di Milano Bovisa

The trajectory was evaluated integrating environment modeling, considering RRT-based path planning, minimum-snap trajectory generation, collision checking, and trajectory validation for a UAV in a 3D environment. It produces a smooth, dynamically feasible trajectory from a start to a goal position while accounting for obstacles and UAV size (through inflation of obstacles). 

## Sources
- Notes from the course `Adaptive and Autonomous Aerospace Systems`
