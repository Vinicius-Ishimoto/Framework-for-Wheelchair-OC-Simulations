# Results Structure

Inside the structure, there are 4 sub-sections as follows:

  - FDATA: All the data gattered from the simulations and the parameters used in the simulations;
  - ReferenceModel: Mass and Friction of the reference model used in the simulations;
  - Simulations: Table with the multipliers of the impedance model in relation to the reference model.
      
      Each row represents a different case: M100-C100, M50-C100, M100-C50 and M50-C50, in this order.

      
## FDATA Structure

FDATA has 2 main structures:

  - Plano: Simulations on the level surface;
  - Rampa: Simulations with a slope angle of 3 degrees;

For each type represented before, there are 2 main types of simulations:

  - Reference: Simulation without the controller;
  - PIController: Simulation with the PI impedance controller. 
   
      Each structure represents the cases: M100-C100, M50-C100, M100-C50 and M50-C50, in this order.

### Simulations Structure

The structure inside each simulation case, has the following properties:

   - Person: Parameters utilized on the model;
   - Options: Options inserted on the function for the simulation;
   - Results: Vector of results from the optimal control over time with all the phases concatenated;
   - Raw: Raw results got from the optimizer;
