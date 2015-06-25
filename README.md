# Forecast controller for gridsim simulator

## Version
This demo is used with gridsim 'release/0.1.1rc2' released February 4, 2015

## Synopsis
This project use gridsim and add some new elements to use the simulator with a main controller and some local forecast controllers. 
The main controller send information to the local controllers such as the temperature forecast and a cost vector forecast to the
local controllers. These last optimize the consumption of their device to minimize the costs. A cost represent the difference
between energy consumption and energy production. The cost is low if consumption < production and tends to be high if production < consumption.

## Library dependencies
* [deap](http://deap.readthedocs.org/en/master/ "deap documentation") is a distributed evolutionary algorithms library for python.
* [PuLP](https://pypi.python.org/pypi/PuLP "PuLP documentation") is a LP modeler using to generate problem for MILP solver such that  GLPK, CBC, CPLEX and Gurobi.

## Solver dependencies
The resolution is done by an integer linear programming solver (ILP). This application support the three solver below. Be sure to have one of them installed on your computer.

* [Gurobi](http://www.gurobi.com/academia/for-universities "gurobi with a free academic license") is a commercial solver but free to use for academic community. It is one of the most performing solver.
* [CPLEX](https://www.ibm.com/developerworks/community/blogs/jfp/entry/cplex_studio_in_ibm_academic_initiative?lang=en "cplex") is another commercial solver developed by IBM. It support free to use academic use. 
* [COIN-OR CBC](https://projects.coin-or.org/Cbc "Cbc project") is a free and open-source solver under Eclipse Public License (considered to be business-friendly free software license).
* [GLPK](http://www.gnu.org/software/glpk/ "GNU Linear Programming Kit") is a free and open-source solver under GNU Public License.

According to some benchmarks<sup>[1](http://www.statistik.tuwien.ac.at/forschung/CS/CS-2012-1complete.pdf "statistik"), 
[2](http://www.gurobi.com/resources/switching-to-gurobi/open-source-solvers "gurobi benchmark")</sup> prefer this order: Gurobi or CPLEX followed by CBC and GLPK. This application was successfully tested with Gurobi, CBC and GLPK. CPLEX was not tested. 

## Using this demo
clone the repository inside the gridsim/demo/ repository and run the running_script file.

## Config this demo
This application use the config.json file as a configuration for the simulation. You can use the config-default.json to
make your own config file. The name of the config file must be "config.json". The config file looks like the format below.

```
{
  "simulation_configuration":{
    "duration_day": 1,
    "decision_duration_day_period": 1,
    "duration_minute_step": 30,
    "weather_degree_slot_sigma":0.2,
    "weather_degree_day_sigma":0.4,
    "cost_sigma": 1,
    "cost_period_slot":
     [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
      2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 2.0, 2.0, 2.0, 2.0,
      5.0, 5.0, 7.0, 7.0, 10.0, 10.0, 10.0, 10.0, 8.0, 8.0,
      4.0, 4.0, 3.0, 3.0, 4.0, 4.0, 6.0, 6.0, 8.0, 8.0,
      5.0, 5.0, 2.0, 2.0, 1.0, 1.0, 0.0, 0.0]
  },

  "solver_configuration":{
    "solvers":["gurobi", "cplex", "cbc", "glpk"],
    "messages": "false",
    "time_limit_second":0
  },

  "device_configuration":{
    "number": 1,
    "initial_outside_coupling": 20,
    "size_min_meter": 20,
    "size_max_meter": 400,
    "power_min_watt": 1000,
    "power_max_watt": 5000,
    "height_min_meter": 3,
    "height_max_meter": 50,
    "hysteresis_user_preference_min": 1,
    "hysteresis_user_preference_max": 3,
    "coupling_min": 40,
    "coupling_max": 90,
    "temperature_min": 18,
    "temperature_max": 24
  }
}
```
This file has three main section: a simulation configuration, a solver configuration and a device configuration.

### Simulation configuration
This section is devoted to the whole simulation. Each field is explain below:

* ```duration_day``` is used to configure the duration of the simulation in days.
* ```decision_duration_day_period``` is the the duration of one decision step. It is the frequency of the optimization 
process made by the local controllers.
* ```duration_minute_slot```  is the duration of one slot/step during the decision duration day period. By example, if a 
decision duration day period is 1 day and the duration minute slot is 30 minutes, the number of slots will be 48. This
computed number (number of slot) will define the complexity of the problem. Each slot need an on/off decision. The
number of decision variables is the number of slots. The number of possible combinations is 2^n, where n is the number
of decision variables. So, if we have a shortest granularity, by example 96 slots a day instead of 48 for a duration minute step of 15min, 
the problem has 2^96 combinations (and not 2*2^48) instead of 2^48. The number of combinations is not linear and 
therefor the computation time won't be either linear. 
* ```weather_degree_slot_sigma``` is the sigma to simulate a difference between weather forecast for each period.
* ```weather_degree_day_sigma``` is the sigma to simulate a difference between weather forecast for each day.
* ```cost_sigma``` is the sigma to simulate a different cost every day according to the cost period step.
* ```cost_period_slot``` is the cost vector reference. Each component of the vector is the cost for one period. If the
number of costs is different than the number of period, a warning will fire but it is possible to use different scales.

### Solver configuration
This section is devoted to the solver configuration:

* ```solvers``` is the list of supported solvers. The order define the solver preference.
* ```messages``` is set to true if the solver print some information.
* ```time_limit_second``` is the time out of the solver. If the solver stop running due to the timeout, the solution 
won't be optimal but suboptimal. In the worst case the solver doesn't find a solution.

### Device configuration
This section is devoted to the device configuration. It this application, we use heat pomp to manage building temperature.

* ```number``` is the number of building with their local controller
* ```size_min_meter``` and ```size_max_meter``` the size of the building is randomly chosen between min and max
* ```power_min_watt``` and ```power_max_watt``` the power of the heat pomp of the building is randomly chosen between min and max
* ```height_min_meter``` and ```height_max_meter``` the height of the building is randomly chosen between min and max
* ```hysteresis_user_preference_min``` and ```hysteresis_user_preference_max``` the height of the building is randomly chosen between min and max
* ```coupling_min``` and ```coupling_max``` the coupling of the building with outdoor
* ```temperature_min``` and ```temperature_max``` the temperature preference for the building

## Mathematical model for the forecast local controller

![Alt text](https://github.com/gridsim/forecastcontroller/blob/documentation/math_model.gif?raw=true "test")

formulas done thanks to [numberempire.com](http://fr.numberempire.com/texequationeditor/equationeditor.php)
