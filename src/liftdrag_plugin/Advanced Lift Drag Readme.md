#Introduction
This document describes the new Gazebo plugin for aerodynamic modeling, updates to the physics formulation (quadratic drag, sideforce, moments about all three axes, body rate derivatives), and also figure out a workflow to get from AVL derivatives to Gazebo parameters, while maintaining accurate physics representation/effects.

#Functional Requirements for Simulator
Gazebo uses an ENU (east-north-up) reference frame, while the AVL equations of motion assume an NED (north-east-down) frame. Both of these frames are mapped onto the aircraft assuming that it is traveling along the X-axis of the frame (east in ENU, north in NED). What this means in practice is that the ENU frame has a Y axis out the left wing and a Z axis pointing upward, while the NED frame has a Y axis pointing out the right wing and a Z axis pointing downward. As such, the angles, forces, and moments need to be translated between the frames.
The SITL needs to meet these requirements:
    1. Compatible with our current joystick configuration
    2. Compatible with the mixer files on the flying wing
    3. Compatible with AVL derivatives
    
#Notation
All the variables representing aerodynamic coefficients and derivatives are named according to the following convention:
    • The first letter of these variable names is a capital C.
    • The next letter(s) represent the force or moment that this variable relates to. 
        ◦ If this variable pertains to a force, the next letter is a capital L, D, or Y, representing Lift, Drag, or sideforce (force in the Y- direction) respectively.
        ◦ If this variable pertains to a moment, the next letters are “ell”, “em”, or “en”, all lowercase, representing roll moment, pitch moment, and yaw moment respectively. This was chosen to avoid confusion between CL, the coefficient of lift, and Cell, the coefficient of roll moment.
    • The next letter(s) represent what the coefficient is being differentiated with respect to.
        ◦ 0: represents zero-angle-of-attack or zero-lift condition (not a differential)
        ◦ a: angle of attack, alpha
        ◦ b: sideslip angle, beta
        ◦ ctrl: control surface deflection
        ◦ p: non-dimensionalized roll rate
        ◦ q: non-dimensionalized pitch rate
        ◦ r: non-dimensionalized yaw rate

One potentially confusing item in the code is the abbreviation “CDp”. CDp could be used to refer to a body rate derivative (the change in coefficient of drag with respect to the non-dimensionalized roll rate) or parasitic drag. This code uses it to refer to the body rate derivative: the parasitic drag (also known as “zero-lift drag”) is referred to as CD0 in the code.

#Coordinate Frames
As of August 2022, the advanced lift-drag plugin computes all forces in the stability frame, and all moments in the body frame. The stability frame is a reference frame oriented such that the X-axis is pointed opposite to the projection of the relative wind vector onto the plane of symmetry of the aircraft. In other words, compared to the body frame, the stability frame has been rotated about the Y-axis by an angle ⍺, the angle of attack. This is important, since the lift and drag are computed in this frame.
A diagram showing the stability, body, and wind axes can be found at: https://aviation.stackexchange.com/questions/70577/under-what-circumstances-is-the-aircraft-stability-axis-system-used-in-either

#User Entered Parameters
From user measurements/calculations:
    1. Reference area, span, chord, aspect ratio
    2. Stall angle
    3. Zero-lift coefficient of drag

From AVL files: 
    1. Zero-angle-of-attack coefficients of lift and pitching moment
    2. Oswald efficiency
        a. AVL’s efficiency is span efficiency, which would be the same as Oswald efficiency, if profile drag does not depend on the square of CL. This is not true in general: as such, the span efficiency is an upper bound on Oswald efficiency.
        b. AVL sometimes computes an incorrect number for span efficiency, and you may need to calculate it by hand. AVL’s drag and lift numbers are correct, however, so plugging those into the following formula should yield the span efficiency.
e =  \frac{{{C}_{L}} ^ {2}}{π AR( {C}_{Di} )}
{C}_{Di} = {C}_{D} - {C}_{D0}
AVL is an inviscid solver, which means it doesn’t have a CD0 added in by default. As such, its CD equals CDi, unless you manually add in a CD0.
Note that span efficiencies greater than 1 can exist; non-planar wings can have span efficiencies greater than 1. However, if AVL comes up with a span efficiency greater than 1 for a planar wing, it is most likely wrong.
    3. Body axis derivatives
    4. Stability axis derivatives
    5. Control derivatives 
    6. Body rate derivatives

For AVL derivatives, the current plan is to use the control derivatives at a single operating point - currently the trim condition (i.e. the angle of attack it would fly at when traveling at a certain “cruise airspeed”). 

Something to note about the AVL derivatives: every one of the angular derivatives is how much the value of some coefficient changes per radian of angle of attack or sideslip angle. The control derivatives, however, are how much that coefficient changes per degree of control surface deflection. This is handled by the code; you can put the numbers from AVL directly in the code. However, this is still worth knowing.

#How To Use AVL
AVL (Athena Vortex Lattice) is a well-known program used to analyze a variety of aircraft. It uses a vortex-lattice method to compute the forces and moments acting on an aircraft at any arbitrary flight condition. It also can compute the stability and control derivatives needed to create a linearized simulation of an aircraft about a trim condition. 
To use AVL, you first need to create a file containing your aircraft configuration. The AVL User Primer (https://web.mit.edu/drela/Public/web/avl/AVL_User_Primer.pdf) explains how to do this. Another useful resource is the variety of example files that comes with AVL: modifying one of those to suit your needs is typically much easier than writing an AVL file from scratch.
Once you have created your aircraft input file, open up AVL, and load your aircraft using the LOAD command. Go to the operations menu with the OPER command; from here, typing in X and hitting the Enter/Return key should execute a flow computation. Once you are satisfied with the computation, use the ST command to get derivatives in the stability axes and the SB commands to get derivatives in the body axes. Note that you will need derivatives from both of these commands to fill out the advanced lift-drag plugin.

One important fact to note. This plugin treats each individual control surface as its own surface; it cannot combine them the way AVL can. As such, if you are trying to determine the control derivatives for your aircraft’s ailerons, you need to determine the control derivatives for the left and the right ailerons individually- doing them both at once may not work.

Make sure to sanity-check your results! While AVL is generally reliable, there are some cases where it has produced erroneous results. For example, when I ran an R/C aircraft after something large (e.g. the Boeing 737 example model), AVL used the wrong reference point; as a result, several derivatives came out enormously larger than they should have.

#Forces and Moments
Using the reference area of the aircraft, the derivatives from AVL, the control surface deflections, the angle of attack and sideslip angle, and the non-dimensionalized body rates, this simulator models the main forces of lift and drag on the vehicle. 
Most of the above parameters are used to calculate the current coefficients of lift, drag, sideforce, and moment about all three axes. 
To compute the aerodynamic coefficients, the code sums up the contributions from several different effects. The following equations hold in the linear, pre-stall flight regime:

{C}_{L}  =  {C}_{L0} + {C}_{Lα} α + {C}_{Lp} p+ {C}_{Lq} q+ {C}_{Lr} r+ {Σ}_{x=1}^{CS} {C}_{L,ctrl,x} {δ}_{ctrl,x}
{C}_{D}  =  {C}_{D0} + \frac{{{C}_{L}} ^ {2}}{π ARe} + {C}_{Dp} p+ {C}_{Dq} q+ {C}_{Dr} r+ {Σ}_{x=1}^{CS} {C}_{D,ctrl,x} {δ}_{ctrl,x}
{C}_{Y}  =  {C}_{Y β} β + {C}_{Yp} p+ {C}_{Yq} q+ {C}_{Yr} r+ {Σ}_{x=1}^{CS} {C}_{Y,ctrl,x} {δ}_{ctrl,x}
{C}_{ℓ}  =  {C}_{ℓ β} β + {C}_{ℓp} p+ {C}_{ℓq} q+ {C}_{ℓr} r+ {Σ}_{x=1}^{CS} {C}_{ℓ,ctrl,x} {δ}_{ctrl,x}
{C}_{m}  =  {C}_{m α} α + {C}_{mp} p+ {C}_{mq} q+ {C}_{mr} r+ {Σ}_{x=1}^{CS} {C}_{m,ctrl,x} {δ}_{ctrl,x}
{C}_{n}  =  {C}_{n β} β + {C}_{np} p+ {C}_{nq} q+ {C}_{nr} r+ {Σ}_{x=1}^{CS} {C}_{n,ctrl,x} {δ}_{ctrl,x}

Here, the expression “#CS” refers to the number of control surfaces on the aircraft. The summation adds up the effect of all the control surfaces.
Post stall, most of these equations have been left unchanged. The ones that do change compute the coefficients of lift and drag, which change to the following:

{C}_{L}  = 2{sin} ^ {2} ( α )cos( α )+ {C}_{Lp} p+ {C}_{Lq} q+ {C}_{Lr} r+ {Σ}_{x=1}^{CS} {C}_{L,ctrl,x} {δ}_{ctrl,x}
{C}_{D}  =  {C}_{D,FP} *(0.5-0.5cos(2 α ))+ {C}_{Dp} p+ {C}_{Dq} q+ {C}_{Dr} r+ {Σ}_{x=1}^{CS} {C}_{D,ctrl,x} {δ}_{ctrl,x}

CD,FP is the flat-plate coefficient of drag. Currently, this is computed as follows:

  {C}_{D,FP} = \frac{2}{1+ {e} ^ {K1+K2*AR}} 

K1 and K2 are empirical coefficients. Currently, K1 is equal to -0.224, and K2 is equal to -0.115.

A sigmoid function is used to blend the pre-stall and post-stall models together, with a blending parameter M used to determine how sharply the plane stalls. Currently, the default value of M is 15, which switches from pre-stall to post-stall over the course of about 1.8 degrees of angle of attack.

σ = \frac{1+ {e} ^ {-M*( α - {α}_{stall} )} +{e} ^ {M*( α - {α}_{stall} )}}{(1+ {e} ^ {-M*( α - {α}_{stall} )} )*( {1+e} ^ {M*( α - {α}_{stall} )} )}

As such, the actual equations for the coefficient of lift and drag are:

{C} _{L}  =(1- σ )*( {C}_{L0} + {C}_{L α} α )+ σ *2{sin} ^ {2} ( α )cos( α )+ {C}_{Lp} p+ {C}_{Lq} q+ {C}_{Lr} r+ {Σ}_{x=1}^{CS} {C}_{L,ctrl,x} {δ}_{ctrl,x}
{C}_{D}  = (1- σ )* ({C}_{D0} + \frac{{{C}_{L}} ^ {2}}{π ARe})+ σ *( {C}_{D,FP} *(0.5-0.5cos(2 α )))+ {C}_{Dp} p+ {C}_{Dq} q+ {C}_{Dr} r+ {Σ}_{x=1}^{#CS} {C}_{D,ctrl,x} {δ}_{ctrl,x}

Given these coefficients, multiplying by the reference area of the aircraft (Sref) and the current dynamic pressure (q) produces the forces.

F =  {C}_{F} {S}_{ref} q
q=1/2 ρ {v} ^ {2}

The moments, however, need an additional term: either the span (b) or the mean aerodynamic chord (c) of the aircraft.

ℓ =  {C}_{ℓ} {S}_{ref} qb
m=  {C}_{m} {S}_{ref} qc
n =  {C}_{n} {S}_{ref} qb

#Post-Stall Models
As of August 2022, the plugin uses two post-stall models to compute the coefficient of lift and drag on the aircraft. The model for lift coefficient is a Newtonian model, based on the Third Law of Motion. 
The post-stall drag model was constructed using two papers: one from Stringer et al which provides drag coefficients at angles of attack from zero to 360 degrees and the other from Ostowari and Naik which provides flat plate coefficients of drag at a variety of aspect ratios. The Stringer paper was used to relate drag to angle of attack, while Ostowari’s report was used to create a model of flat plate drag at a variety of aspect ratios. To create the latter model, I fit a sigmoid function to some of the numbers in the second paper, with the form below.

{C}_{D,fp} = \frac{2}{1+ {e} ^ {K1+K2*AR}}

The coefficients K1 and K2 are in the code, but might need some tuning.
A sigmoid function was used because CD initially increases quickly as aspect ratio rises, but that increase would slow down as AR goes to infinity. Intuitively, it seemed like a good model for the drag vs. aspect ratio relation, and the fit seemed pretty good as well.
Looking at the data from Stringer, I could see that the CD was approximately equal to 1-cos(2*AoA). This has a maximum value of 2, so I divided this expression by 2 to make sure multiplying it by flat-plate drag would not produce unreasonable results. 
The resulting drag model uses the flat plate model to determine an upper bound on coefficient of drag, then multiplies that upper bound by 0.5-0.5*cos(2*AoA) to determine the actual coefficient of drag. This should provide at least a usable estimate of post stall drag.
