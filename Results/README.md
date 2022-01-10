# Results Structure

Inside the structure, there are 4 sub-sections as follows:

  - FDATA: All the data gattered from the simulations and the parameters used in the simulations;
  - ReferenceModel: Mass and Friction of the reference model used in the simulations;
  - Simulations: Table with the multipliers of the impedance model in relation to the reference model.
      
      Each row represents a different case: M100-C100, M50-C100, M100-C50 and M50-C50, in this order.

      
## FDATA Structure

FDATA has 2 main structures:

  - Plano: Simulations on the level surface (horizontal);
  - Rampa: Simulations with a slope angle of 3 degrees;

For each type represented before, there are 2 main types of simulations:

  - Reference: Simulation without the controller;
  - PIController: Simulation with the PI impedance controller. 
   
      Each structure represents the cases: M100-C100, M50-C100, M100-C50 and M50-C50, in this order.

The sub-structure inside each simulation case, has the following fields:

   - Person: Parameters utilized on the model;
   - Options: Options inserted on the function for the simulation;
   - Results: Vector of results from the optimal control over time with all the phases concatenated;
   - Raw: Raw results got from the optimizer;

Examples:

    PersonData = FDATA.Plano.PIController{2}.Person % User/Wheelchair parameters for the simulation on level surface with controller case M50-C100
    Results = FDATA.Rampa.Reference.Results % Results concatenated for the simulation on a slope angle without controller
    
# Wheelchair/Person Parameters Field

Parameters shown in the .Person field. Some of the parameters used in the simulations are explained below. Due to the simetry adopted, the inertial parameters represents the data of both arms:

  - ma, Ja, a, A: mass, inertia, distance from the shoulder to the center of mass and total length of the upperarms;
  - mb, Jb, b, B: mass, inertia, distance from the elbow to the center of mass and total length of the forearms;
  - mr, Jr: mass and inertia of the rear wheels of the wheelchair
  - R, Rr: Pushrim and wheel radious;
  - Mf, Jf: total mass and inertia of the system;
  - h, Y: horizontal and vertical distance between the shoulders and the center of the rear wheels of the wheelchair;
  - dFric, Frr, Ang: friction coeficient, roling resistance and slope angle;
  - Mi, Ci, Fri: mass, friction coeficient and rolling resistance of the impedance controller;
  - thetac1, thetac2: critical values of theta. Values where the the arms are full extended in the propulsion phase;
  - const: constant of the controller;

The other values in the function are deprecated, or only used as reference values.

# Results Field

Here, the results from the simulations are concatenated over time, some of the main data are:

  - time: time vector
  - alpha, dalpha: angle and angular velocity between the upperarm and the direction of motion;
  - beta, dbeta: angle and angular velocity between the forearm and the direction of motion;
  - theta, dtheta: angle and angular velocity of the rear wheels of the wheelchair. The initial value of the angle is the initial contact in the propulsion phase;
  - taup: tangencial torque applied by the user;
  - act_se, act_sf, act_ee, act_ef: muscle activation of the shoulder extension and flexion and elbow extension and flexion respectively;
  - dct_se, dct_sf, dct_ee, dct_ef: neural excitation of the shoulder extension and flexion and elbow extension and flexion respectively;
  - tau_se, tau_sf, tau_ee, tau_ef: torque related to the shoulder extension and flexion and elbow extension and flexion respectively;
  - tau_sp, tau_ep: passive torque of the shoulder and elbow respectively;
  - x: total displacement of the wheelchair;
