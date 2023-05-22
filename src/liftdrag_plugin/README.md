<h1>Introduction</h1><br>This document describes the the advanced_liftdrag_plugin. Primary modeling differences from the liftdrag_plugin include:
- quadratic formulation for drag
- side force
- flat-plate post-stall model
- aerodynamic moments about all three axes
- body rate stability derivatives
- actuator control derivatives
<h1>Notation</h1>
All the variables representing aerodynamic coefficients and derivatives are named according to the following convention: <br>
<ul>
    <li>The first letter of these variable names is a capital C.</li>
    <li>The next letter(s) represent the force or moment that this variable relates to.</li>
    <ul>
        <li>If this variable pertains to a force, the next letter is a capital L, D, or Y, representing <b>L</b>ift, <b>D</b>rag, or sideforce (force in the <b>Y</b>- direction) respectively.</li>
        <li>If this variable pertains to a moment, the next letters are “ell”, “em”, or “en”, all lowercase, representing roll moment, pitch moment, and yaw moment respectively. This was chosen to avoid confusion between CL, the coefficient of lift, and Cell, the coefficient of roll moment.</li>
    </ul>
    <li>The next letter(s) represent what the coefficient is being differentiated with respect to.</li>
    <ul>
        <li>0: represents the respective coefficient with zero angle of attack (in the case of lift and pitching moment) or zero lift (in the case of drag), zero body rates, and zero actuator deflection (not a differential)</li>
        <li>a: angle of attack, alpha</li>
        <li>b: sideslip angle, beta</li>
        <li>ctrl: control surface deflection</li>
        <li>$\hat{p}$: non-dimensionalized roll rate $(\hat{p} = \frac{pB}{2V})$</li>
        <li>$\hat{q}$: non-dimensionalized pitch rate $(\hat{p} = \frac{q\overline{c}}{2V})$</li>
        <li>$\hat{r}$: non-dimensionalized yaw rate $(\hat{r} = \frac{rB}{2V})$</li>
    </ul>
 </ul>
In the definitions of the non-dimensionalized rates above, $\hat{p},\hat{q},$ and $\hat{r}$ refer to the non-dimensional rates, p,q, and r to the dimensional rates, B to the wingspan (capitalized to avoid confusion with the lowercase b used in the naming convention), and $\overline{c}$ to refer to the mean aerodynamic chord.
One potentially confusing item in the code is the abbreviation “CDp”. CDp could be used to refer to a body rate derivative (the change in coefficient of drag with respect to the non-dimensionalized roll rate) or parasitic drag. This code uses it to refer to the body rate derivative: the parasitic drag (also known as “zero-lift drag”) is referred to as CD0 in the code. <br>
<h1>Coordinate Frames</h1>
Gazebo uses an ENU (east-north-up) reference frame, while the AVL equations of motion assume an NED (north-east-down) frame. Both of these frames are mapped onto the aircraft assuming that it is traveling along the X-axis of the frame (east in ENU, north in NED). What this means in practice is that the ENU frame has a Y axis out the left wing and a Z axis pointing upward, while the NED frame has a Y axis pointing out the right wing and a Z axis pointing downward. As such, the angles, forces, and moments need to be translated between the frames.
The advanced lift-drag plugin computes all forces in the stability frame, and all moments in the body frame. The stability frame is a reference frame oriented such that the X-axis is pointed opposite to the projection of the relative wind vector onto the plane of symmetry of the aircraft. In other words, compared to the body frame, the stability frame has been rotated about the Y-axis by an angle ⍺, the angle of attack. This is important, since the lift and drag are computed in this frame. Further explanation and a diagram can be found in section 16.2 of Raymer's Aircraft Design: A Conceptual Approach. <br>
<h1>Getting the Aerodynamic Parameters</h1>
<h2>AVL</h2>
AVL, also known as Athena Vortex Lattice, was used to obtain the aerodynamic parameters and derivatives needed for the advanced lift-drag plugin. AVL uses a vortex-lattice method to compute the forces and moments acting on an aircraft at any arbitrary flight condition. These forces are computed at a reference point, defined in the input file. This reference point is defined in the NED coordinate frame with the origin at the nose of the aircraft. AVL can also compute the stability and control derivatives needed to create a linearized simulation of an aircraft about a trim condition.<br> To use AVL, you first need to create a file containing your aircraft configuration. The AVL User Primer (https://web.mit.edu/drela/Public/web/avl/AVL_User_Primer.pdf) explains how to do this. Another useful resource is the variety of example files that comes with AVL: modifying one of those to suit your needs is typically much easier than writing an AVL file from scratch. Note that AVL does not use units- it assumes they are consistent in the input file. Make sure you are careful with your units. <br> Once you have created your aircraft input file, open up AVL, and load your aircraft using the LOAD command. Go to the operations menu with the OPER command; from here, typing in X and hitting the Enter/Return key should execute a flow computation. Once you are satisfied with the computation, use the ST command to get derivatives in the stability axes and the SB commands to get derivatives in the body axes. Note that you will need derivatives from both of these commands to fill out the advanced lift-drag plugin. <br> One important fact to note. This plugin treats each individual control surface as its own surface; it cannot combine them the way AVL can. As such, if you are trying to determine the control derivatives for your aircraft’s ailerons, you need to determine the control derivatives for the left and the right ailerons individually- doing them both at once may not work. <br> <br>Make sure to sanity-check your results! While AVL is generally reliable, there are some cases where it has produced erroneous results. For example, when running an R/C aircraft after something large (e.g. the Boeing 737 example model), AVL used the wrong reference point, resulting in several derivatives coming out enormously larger than they should have. The reference point of the R/C aircraft was 0.1 m (4 in) behind the nose, while that of the Boeing is 60 ft (18 m) back. Furthermore, the R/C aircraft file uses meters while the Boeing uses feet, which meant that AVL interpreted the Boeing's reference point location as 60 m back of the nose. 
<h2>Necessary Parameters</h2>
From user measurements or calculations:
<ol>
   <li>Reference area, span, chord, aspect ratio</li>
   <li>Stall angle</li>
   <li>Zero-lift coefficient of drag</li>
</ol>
From AVL files:
<ol>
    <li>Zero-angle-of-attack coefficients of lift and pitching moment</li>
    <li>Oswald efficiency</li>
    <li>Body axis derivatives</li>
    <li>Stability axis derivatives</li>
    <li>Control derivatives</li>
    <li>Body rate derivatives</li>
</ol>
When using AVL to calculate the wing efficiency, remember that AVL’s efficiency is span efficiency, which would be the same as Oswald efficiency, if profile drag does not depend on the square of CL. This is not true in general: as such, the span efficiency is an upper bound on Oswald efficiency. <br> AVL sometimes computes an incorrect number for span efficiency, and you may need to calculate it by hand. AVL’s drag and lift numbers are correct, however, so plugging those into the formula $e = \frac{C_L^2}{\pi C_{Di} AR}$ should yield the span efficiency. The coefficient of induced drag, $C_{Di}$, is calculated by subtracting the zero lift drag from the total drag: $C_{Di} = C_D-C_{D0}$. AVL is an inviscid solver, which means it doesn’t have a CD0 added in by default. As such, its CD equals CDi, unless you manually add in a CD0. Note that span efficiencies greater than 1 can exist; non-planar wings can have span efficiencies greater than 1. However, if AVL comes up with a span efficiency greater than 1 for a planar wing, it is most likely wrong.<br>
For AVL derivatives, the current plan is to use the control derivatives at a single operating point - currently the trim condition (i.e. the angle of attack it would fly at when traveling at a certain “cruise airspeed”). <br> Something to note about the AVL derivatives: every one of the angular derivatives is how much the value of some coefficient changes per radian of angle of attack or sideslip angle. The control derivatives, however, are how much that coefficient changes per degree of control surface deflection. You can put the numbers from AVL directly in the code, since the code converts the control surface deflections from radians to degrees. <br> 
<h1>Forces and Moments</h1><br>
Using the reference area of the aircraft, the derivatives from AVL, the control surface deflections, the angle of attack and sideslip angle, and the non-dimensionalized body rates, this simulator models the main forces of lift and drag on the vehicle. 
Most of the above parameters are used to calculate the current coefficients of lift, drag, sideforce, and moment about all three axes. 
To compute the aerodynamic coefficients, the code sums up the contributions from several different effects. The following equations hold in the linear, pre-stall flight regime:

$$C_L  =  C_{L0} + C_{Lα} α + C_{Lp} p+ C_{Lq} q+ C_{Lr} r+ \sum_{x=1}^{CS} C_{L,ctrl,x} \delta_{ctrl,x}$$
$$C_D  =  C_{D0} + \frac{{{C_L}} ^ {2}}{π ARe} + C_{Dp} p+ C_{Dq} q+ C_{Dr} r+ \sum_{x=1}^{CS} C_{D,ctrl,x} \delta_{ctrl,x}$$
$$C_Y  =  C_{Y β} β + C_{Yp} p+ C_{Yq} q+ C_{Yr} r+ \sum_{x=1}^{CS} C_{Y,ctrl,x} \delta_{ctrl,x}$$
$$C_ℓ  =  C_{ℓ β} β + C_{ℓp} p+ C_{ℓq} q+ C_{ℓr} r+ \sum_{x=1}^{CS} C_{ℓ,ctrl,x} \delta_{ctrl,x}$$
$$C_m  =  C_{m α} α + C_{mp} p+ C_{mq} q+ C_{mr} r+ \sum_{x=1}^{CS} C_{m,ctrl,x} \delta_{ctrl,x}$$
$$C_n  =  C_{n β} β + C_{np} p+ C_{nq} q+ C_{nr} r+ \sum_{x=1}^{CS} C_{n,ctrl,x} \delta_{ctrl,x}$$

Here, the expression “CS” refers to the number of control surfaces on the aircraft. The summation adds up the effect of all the control surfaces.
Post stall, most of these equations have been left unchanged. The ones that do change compute the coefficients of lift and drag, which change to the following:

$$C_L = 2{sin} ^ {2} ( α )cos( α )+ C_{Lp} p+ C_{Lq} q+ C_{Lr} r+ \sum_{x=1}^{CS} C_{L,ctrl,x} \delta_{ctrl,x}$$
$$C_D = C_{D,FP} (0.5-0.5cos(2 α ))+ C_{Dp} p+ C_{Dq} q+ C_{Dr} r+ \sum_{x=1}^{CS} C_{D,ctrl,x} \delta_{ctrl,x}$$

$C_{D,FP}$ is the flat-plate coefficient of drag. Currently, this is computed as follows:
  $C_{D,FP} = \frac{2}{1+ e ^ {K1+K2 AR}}$

K1 and K2 are empirical coefficients. Currently, K1 is equal to -0.224, and K2 is equal to -0.115. <br> A sigmoid function is used to blend the pre-stall and post-stall models together, with a blending parameter M used to determine how sharply the plane stalls. Currently, the default value of M is 15, which switches from pre-stall to post-stall over the course of about 1.8 degrees of angle of attack.

$$\sigma = \frac{1+ e ^ {-M*( \alpha - \alpha_{stall} )} +e ^ {M*( \alpha - \alpha_{stall} )}}{(1+ e ^ {-M*( \alpha - \alpha_{stall} )} ) ( {1+e} ^ {M( α - α_{stall} )} )}$$

As such, the actual equations for the coefficient of lift and drag are:

$$C_L  =(1- \sigma )( C_{L0} + C_{L \alpha} \alpha )+ 2 \sigma {sin} ^ 2 ( α )cos( α )+ C_{Lp} p+ C_{Lq} q+ C_{Lr} r+ \sum_{x=1}^{CS} C_{L,ctrl,x} \delta_{ctrl,x}$$
$$C_D = (1-\sigma)(C_{D0}+\frac{{{C_L}} ^ {2}}{π ARe})+ \sigma C_{D,FP}(0.5-0.5cos(2 α ))+ C_{Dp} p+ C_{Dq} q+ C_{Dr} r+ \sum_{x=1}^{CS} C_{D,ctrl,x} \delta_{ctrl,x}$$

Given these coefficients, multiplying by the reference area of the aircraft (Sref) and the current dynamic pressure $(\overline{q})$ produces the forces.

$$F =  C_F S_{ref} \overline{q}$$
$$\overline{q}=\frac{1}{2} \rho v ^ {2}$$

Here, $\rho$ refers to air density. The moments, however, need an additional term: either the span (b) or the mean aerodynamic chord $(\overline{c})$ of the aircraft.

$$ℓ =  C_ℓ S_{ref} \overline{q}B$$
$$m=  C_m S_{ref} \overline{q}\overline{c}$$
$$n =  C_n S_{ref} \overline{q}B$$

<h1>Post-Stall Models</h1>
The plugin uses two post-stall models to compute the coefficient of lift and drag on the aircraft. The model for lift coefficient is a Newtonian model, based on the Third Law of Motion. 
The post-stall drag model was constructed using two papers: one from Stringer et al (https://aip.scitation.org/doi/pdf/10.1063/1.5011207) which provides drag coefficients at angles of attack from zero to 360 degrees and the other from Ostowari and Naik (https://www.nrel.gov/docs/legosti/old/2559.pdf) which provides flat plate coefficients of drag at a variety of aspect ratios. The Stringer paper was used to relate drag to angle of attack, while Ostowari’s report was used to create a model of flat plate drag at a variety of aspect ratios. To create the latter model, a sigmoid function is fit to some of the numbers in the second paper, with the form below.

$$C_{D,fp} = \frac{2}{1+ e ^ {K1+K2 AR}}$$

The coefficients K1 and K2 are in the code, but might need some tuning.
A sigmoid function was used because CD initially increases quickly as aspect ratio rises, but that increase would slow down as AR goes to infinity. Intuitively, it seemed like a good model for the drag vs. aspect ratio relation, and the fit seemed pretty good as well.
Looking at the data from Stringer, one sees that the CD was approximately equal to 1-cos(2α). This has a maximum value of 2, so this expression was divided by 2 to make sure multiplying it by flat-plate drag would not produce unreasonable results. 
The resulting drag model uses the flat plate model to determine an upper bound on coefficient of drag, then multiplies that upper bound by 0.5-0.5cos(2α) to determine the actual coefficient of drag. This should provide at least a usable estimate of post stall drag.
