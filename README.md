# Synopsis
This project use gridsim and add some new elements to use the simulator with a main controller and some local forecast controllers. 
The main controller send information to the local controllers such as the temperature forecast and a cost vector forecast to the
local controllers. These last optimize the consumption of their device to minimize the costs. A cost represent the difference
between energy consumption and energy production. The cost is low if consumption < production and tends to be high if production < consumption.

# Version
This demo is used with gridsim 'release/0.1.1rc2' released February 4, 2015


# Nomenclature:
 * Decision operation is similar to optimization process
 * Period is the biggest unit of time related to a decision operation
 * Slot (or step) is the smallest unit of time using for iteration. Each slot correspond to a decision (on/off)
 
# Library dependencies
* [deap](http://deap.readthedocs.org/en/master/ "deap documentation") is a distributed evolutionary algorithms library for python.
* [PuLP](https://pypi.python.org/pypi/PuLP "PuLP documentation") is a LP modeler using to generate problem for MILP solver such that  GLPK, CBC, CPLEX and Gurobi.

# Solver dependencies
The resolution is done by an integer linear programming solver (ILP). This application support the three solver below. Be sure to have one of them installed on your computer.

* [Gurobi](http://www.gurobi.com/academia/for-universities "gurobi with a free academic license") is a commercial solver but free to use for academic community. It is one of the most performing solver.
* [CPLEX](https://www.ibm.com/developerworks/community/blogs/jfp/entry/cplex_studio_in_ibm_academic_initiative?lang=en "cplex") is another commercial solver developed by IBM. It support free to use academic use. 
* [COIN-OR CBC](https://projects.coin-or.org/Cbc "Cbc project") is a free and open-source solver under Eclipse Public License (considered to be business-friendly free software license).
* [GLPK](http://www.gnu.org/software/glpk/ "GNU Linear Programming Kit") is a free and open-source solver under GNU Public License.

According to some benchmarks<sup>[1](http://www.statistik.tuwien.ac.at/forschung/CS/CS-2012-1complete.pdf "statistik"), 
[2](http://www.gurobi.com/resources/switching-to-gurobi/open-source-solvers "gurobi benchmark")</sup> prefer this order: Gurobi or CPLEX followed by CBC and GLPK. This application was successfully tested with Gurobi, CBC and GLPK. CPLEX was not tested. 

# Using this demo
clone the repository inside the gridsim/demo/ repository and run the running_script file.

# Config this demo
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

## Simulation configuration
This section is devoted to the whole simulation. Each field is explain below:

* ```duration_day``` is used to configure the duration of the simulation in days.
* ```decision_duration_day_period``` is the the duration of one decision step. It is the frequency of the optimization 
process made by the local controllers.
* ```duration_minute_slot```  is the duration of one slot/step during the decision duration day period. By example, if a 
decision duration day period is 1 day and the duration minute slot is 30 minutes, the number of slots will be 48. This
computed number (number of slot) will define the complexity of the problem. Each slot need an on/off decision. The
number of decision variables is the number of slots. The number of possible combinations is 2<sup>n</sup>, where n is the number
of decision variables. So, if we have a shortest granularity, by example 96 slots a day instead of 48 for a duration minute step of 15min, 
the problem has 2<sup>96</sup> combinations (and not 2*2<sup>48</sup>) instead of 2<sup>48</sup>. The number of combinations is not linear and 
therefor the computation time won't be either linear. 
* ```weather_degree_slot_sigma``` is the sigma to simulate a difference between weather forecast for each period.
* ```weather_degree_day_sigma``` is the sigma to simulate a difference between weather forecast for each day.
* ```cost_sigma``` is the sigma to simulate a different cost every day according to the cost period step.
* ```cost_period_slot``` is the cost vector reference. Each component of the vector is the cost for one period. If the
number of costs is different than the number of period, a warning will fire but it is possible to use different scales.

## Solver configuration
This section is devoted to the solver configuration:

* ```solvers``` is the list of supported solvers. The order define the solver preference.
* ```messages``` is set to true if the solver print some information.
* ```time_limit_second``` is the time out of the solver. If the solver stop running due to the timeout, the solution 
won't be optimal but suboptimal. In the worst case the solver doesn't find a solution.

## Device configuration
This section is devoted to the device configuration. It this application, we use heat pomp to manage building temperature.

* ```number``` is the number of building with their local controller
* ```size_min_meter``` and ```size_max_meter``` the size of the building is randomly chosen between min and max
* ```power_min_watt``` and ```power_max_watt``` the power of the heat pomp of the building is randomly chosen between min and max
* ```height_min_meter``` and ```height_max_meter``` the height of the building is randomly chosen between min and max
* ```hysteresis_user_preference_min``` and ```hysteresis_user_preference_max``` the height of the building is randomly chosen between min and max
* ```coupling_min``` and ```coupling_max``` the coupling of the building with outdoor
* ```temperature_min``` and ```temperature_max``` the temperature preference for the building

## Mathematical model for the forecast local controller
We explain now the mathematical model used by the forecast local controller which is sent to a ILP solver.


![Alt text](https://github.com/gridsim/forecastcontroller/blob/documentation/math.png?raw=true "math model")

What we need is a power on/off decision for each slot of the decision period. Each decision step, the controller need
to solve this model. The main controller send the cost vector for each slot and the weather temperature forecast. For
each device, we need data such as thermal capacity, thermal conductivity, efficiency, hysteresis, temperature reference, 
delta time in seconds for a slot, the initial temperature of the building, the penalty for a temperature out of the hysteresis bound for a slot and
penalty for a underestimate average temperature 

Ex: if we want a 20° C with 2° hysteresis we can keep a temperature between 19° and 21° and globally, the average must be 20°. If
a temperature during the slot <i>s</i> is out of the bound, we have a <i>E</i> penalty multiply by the temperature at slot <i>s</i>. If the average temperature during the period
is lower than 20° the penalty will be <i>D</i> multiply by the difference.

## Explanation
The optimization function <i>(1)</i> minimize the overall cost and penalty as discussed above.
 
* The average of temperature must be the reference temperature. If we are below, we consider a penalty <i>(2)</i>.
* With <i>(3)</i> and <i>(4)</i> we compute the inside and outside temperature leverage. Final temperature for a slot is the sum of leverages <i>(5)</i>.
* Exceeding temperature for each slot are computed thanks to <i>(6)</i> and <i>(7)</i>.
* We initialize the first slot inside temperature according to the real temperature of the building during the resolution process <i>(8)</i>.
* Each inside initial temperature of one slot is the same as the final inside temperature of the slot before <i>(9)</i>.

# Output files for statistics
Five png files are exported into the output directory indicating some information about the simulation:

* <i>average_temperature.png</i> indicates the average temperature for each period through the time
* <i>cost.png</i> the cost function for every period through the time
* <i>error.png</i> the temperature error through the time
* <i>heater.png</i> the on/off decision for each slot <i>(in Joule)</i>

# Code
## agregator.py

This file implement the `AgregatorSimulator` which is a module for gridsim. This represent a central
controller which manage a set of local controllers implemented by `AgregatorElement`.

The central controller send information to the local controllers : the cost and the temperature forecast
in degree for each slot duration.

During the elapsing time, temperature could be different from the forecast. It leads the local controllers to
difference of value. The local controllers can detect such an errors and recompute the decision. The central controllers
could send new costs vector during the simulation, different from the forecast. In this case, the local controllers
need to recompute their decision, too. The decision of an local controller is an on/off decision for each duration slot.

The `ElectroThermalHeaterCooler` represent a model of a heat pomp (heater, cooler, ...) made by Michael Clausen.
It was used by the first version of Agreflex and is not present in Gridsim anymore. I import it for this demo.

The `AgregatorSimulator` is the main controller used as a gridsim module. The local controllers must be register
to it. This module send weather and cost forecast once by period to the local controllers and correct in real time the
cost if there is any change

The `AgregatorElement` is an intermediate abstract class to receive the new cost and detect this receipt to
recompute

The `ForecastController` is a forecast controller. This class optimizes a thermal process given to a
weather forecast and a cost vector.

This class is the main implementation of the algorithms used to improve power balance and power efficiency.
Regarding to a forecast cost and forecast weather, this local controller improve on/off operations. It computes
the day-ahead optimization and correct computation in real time if a new cost or if the temperature forecast is
too different of the real values. A decision is a switch on/switch off for each slot period.

### Decision operation:
Each period, at the beginning of the day by example, we receive cost and weather forecast. This is the day-ahead
operation. We first compute for each slot period if a device should operate or not. We recompute if during the day
we receive a new cost or if an error is detected.
To compute the on/off decision we use linear programming with integer/boolean variables. The decision to switch
on or switch off a device is a variable decision. We have a decision variable for each time slot of the day. The
linear solver provide the on/off result. The data input is the cost for each slot, the power consumption of the
device, the temperature for each slot. The optimization function aims to minimize the cost and to shift the
device operation. It means to decide when the device should run or not.
We assume using heat pomp as devices. It's the most difficult device to manage and the model is generic enough to
simulate any kind of device other than heat pomp (batteries, interrupting devices, white appliances, ...).
Each slot decision is a state which influence the next slot state, hence the difficulty.

### Error Management:
There is some kind of errors.

 * Human behavior: The temperature of a room is different that expected due to a human behavior.  By example,
   someone force to switch on a radiator. The error can be detected and we can re-compute the decision operation
   according to the new values.
 * Configuration settings errors: The values set is wrong. By example, the thermal capacity, the coupling or
   something else lead computations to wrong values. The error can be detected and we re-compute correction
   settings value according to a historic.
 * Weather forecast: The forecast sent at the beginning of a period is different that the real value. If the
   error is too big and misleading results we need to act. The error can be detected according to the historic
   of real temperature and the forecast temperature. If the error is due to a difference, we compute the
   difference value between the forecast and the real value and we recompute the decision operation according
   to this difference.
 * If the user of the simulator add an adjacency room to the actual, computation lead to wrong values due to
   the influence of this room. So we need to consider this "foreign" room.

The main difficulties with error management is to detect what kind of error it is. We start detecting if an
error is due to weather forecast then if it's due to a human behavior. If this is not the case we assume that it
is a configuration settings error. To detect an adjacency room settings, we use a meta-heuristic named
evolutionary/genetic algorithms with the historic of values. The library used is deap.