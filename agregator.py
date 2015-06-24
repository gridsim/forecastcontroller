# -*- coding: utf-8 -*-
"""
.. codeauthor:: Joel Cavat <joel.cavat@hesge.ch>

Nomenclature:
 * Decision operation is similar to optimization process
 * Period is the biggest unit of time related to a decision operation
 * Slot (or step) is the smallest unit of time using for iteration. Each slot correspond to a decision (on/off)

This file implement the :class:`AgregatorSimulator` which is a module for gridsim. This represent a central
controller which manage a set of local controllers implemented by :class:`AgregatorElement`.

The central controller send information to the local controllers : the cost and the temperature forecast
in degree for each slot duration.

During the elapsing time, temperature could be different from the forecast. It leads the local controllers to
difference of value. The local controllers can detect such an errors and recompute the decision. The central controllers
could send new costs vector during the simulation, different from the forecast. In this case, the local controllers
need to recompute their decision, too.

The decision of an local controller is an on/off decision for each duration slot.

The :class:`ElectroThermalHeaterCooler` represent a model of a heat pomp (heater, cooler, ...) made by Michael Clausen.
It was used by the first version of Agreflex and is not present in Gridsim anymore. I import it for this demo.

The :class:`AgregatorSimulator` is the main controller used as a gridsim module. The local controllers must be register
to it. This module send weather and cost forecast once by period to the local controllers and correct in real time the
cost if there is any change

The :class:`AgregatorElement` is an intermediate abstract class to receive the new cost and detect this receipt to
recompute

The :class:`ForecastController` is a forecast controller. This class optimizes a thermal process given to a
weather forecast and a cost vector.

This class is the main implementation of the algorithms used to improve power balance and power efficiency.
Regarding to a forecast cost and forecast weather, this local controller improve on/off operations. It computes
the day-ahead optimization and correct computation in real time if a new cost or if the temperature forecast is
too different of the real values. A decision is a switch on/switch off for each slot period.

Decision operation:
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

Error Management:
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

"""

import sys
import os
sys.path.append(os.path.abspath("../"))

from gridsim.simulation import Simulator, AbstractSimulationElement, Position
from gridsim.controller import ControllerSimulator, AbstractControllerElement
from gridsim.thermal.element import TimeSeriesThermalProcess
from gridsim.thermal.core import ThermalProcess, ThermalCoupling
from gridsim.unit import units
from gridsim.electrical.core import AbstractElectricalCPSElement

# Library used for making linear problems
import pulp
import random

# Used for statistics
import numpy

# Library used for evolutionary/genetic algorithms
from deap import base
from deap import creator
from deap import tools
from deap import algorithms

DEBUG = False


def debug(msg):
    if DEBUG is True:
        print msg


class ElectroThermalHeaterCooler(AbstractElectricalCPSElement):
    """
    Class made by Michael Clausen used in the first version of Agreflex. This class wasn't used in gridsim anymore.
    I import it and use the same mathematical model to simulate heat pomp (heater, cooler,...).
    """
    def __init__(self, friendly_name, pwr, efficiency_factor, thermal_process):
        """
        Electrical heat or cool element. If the efficiency factor is more than 0 the element heats, if less than 0 the
        element cools down the associated thermal process. You can even simulate heat pumps by setting the efficiency
        factor to values greater than 1 or smaller than -1.

        :param: friendly_name: User friendly name to give to the element.
        :type friendly_name: str, unicode
        :param: power: Electrical power of the heating/cooling element.
        :type: float, int
        :param: efficiency_factor: The efficiency factor [], 1.0 means a heater with 100% efficiency and -1 means a
            cooler with 100% efficiency.
        :type: float, int
        :param: thermal_process: Reference to the thermal process where to put/take the energy in/out to heat/cool.
        :type: thermal_process: ThermalProcess
        """
        super(ElectroThermalHeaterCooler, self).__init__(friendly_name)

        if not isinstance(efficiency_factor, (float, int)):
            raise TypeError('efficiency_factor must be a float or int!')
        self._efficiency_factor = float(efficiency_factor)

        if not isinstance(thermal_process, ThermalProcess):
            raise TypeError('thermal_process must be of type ThermalProcess!')
        self._thermal_process = thermal_process

        self.power = pwr

        self._on = False
        """
        Controls the heater/cooler. If this is True, the heater/cooler is active
        and takes energy from the electrical
        network to actually heat or cool the thermal process associated.
        """

        self.history = {}
        """
        Keep a history of values taken by self.on
        """

    @property
    def on(self):
        """
        Getter for parameter on/off

        :return: The value of the parameter. True, device is running, False, device is down
        :rtype: Boolean
        """
        return self._on

    @on.setter
    def on(self, on_off):
        """
        Setter for parameter on/off
        :param on_off: define if the device must work or not
        """
        self._on = on_off

    # AbstractSimulationElement implementation.
    def reset(self):
        """
        Default reset operation
        """
        super(ElectroThermalHeaterCooler, self).reset()
        self.on = False

    def calculate(self, time, delta_time):
        """
        Default calculate operation. We keep an history of operations
        """
        self._internal_delta_energy = units.value(units.convert(self.power * delta_time * units.second, units.joule))

        if not self.on:
            self._internal_delta_energy = 0

        self.history[time] = self.on

    def update(self, time, delta_time):
        """
        Default update operation
        """
        super(ElectroThermalHeaterCooler, self).update(time, delta_time)

        self._thermal_process.add_energy(
            self._delta_energy * self._efficiency_factor
        )

        # HACK: Force to update the thermal process
        self._thermal_process.update(time, delta_time)


class AgregatorSimulator(ControllerSimulator):
    """
    This represent a central controller which manage a set of local controllers
    implemented by :class:`AgregatorElement`.

    The central controller send information to the local controllers : the cost and the temperature forecast
    in degree for each slot duration.

    The cost vector must be set. We apply a variation on this cost to simulate a different cost every days. We apply
    a variation on the real weather sent to simulate the forecast, too
    """

    # Used to simulate a forecast controller according the real value
    PERIOD_SIGMA = 0.24
    DAY_SIGMA = 0.4
    COST_SIGMA = 1

    def __init__(self):
        """
        Simulation module constructor

        """
        super(AgregatorSimulator, self).__init__()
        self._decision_time = 0

        self.friendly_name = "Forecast Temperature"

        self.outside_process = None
        self.outside_temperature = {}
        self.temperature = 0
        self.temperature_outside_forecast = {}
        self.cost_reference = [0]

        self.period_sigma = AgregatorSimulator.PERIOD_SIGMA
        self.day_sigma = AgregatorSimulator.DAY_SIGMA
        self.cost_sigma = AgregatorSimulator.COST_SIGMA

    @property
    def decision_time(self):
        """
        Getter for the decision time

        :return: the decision time

        """
        return self._decision_time

    @decision_time.setter
    def decision_time(self, value):
        """
        Setter for the decision time

        :param value: the decision time

        """
        self._decision_time = units.value(units.convert(value, units.second))

    def attribute_name(self):
        """
        Refer to the module name

        :return: The module name

        """
        return 'agregator'

    def add(self, element):
        """
        Adds the control element to the controller simulation module.

        :param element: Element to add to the control simulator module.
        :type element: AgregatorElement

        """
        if isinstance(element, AgregatorElement):
            element.id = len(self._controllers)
            self._controllers.append(element)
            return element
        return None

    def calculate(self, time, delta_time):
        """
        Method where the temperature forecast is computed and sent to the local controllers and where
        the cost is computed ans sent to the local controller, too

        :param time: The actual time reference
        :type time

        :param delta_time: The delta time
        :type time

        """

        time = units.value(units.convert(time, units.second))
        delta_time = units.value(units.convert(delta_time, units.second))

        # If this is the decision time we simulate a cost and a forecast for the temperature.
        if int(time) % int(self._decision_time) == 0:

            ratio = float(len(self.cost_reference)) / float(len(range(int(time), int(time + self._decision_time), int(delta_time))))
            print "ratio: ", ratio
            j = 0
            cost = {}
            for i in range(int(time), int(time + self._decision_time), int(delta_time)):
                cost[i] = max(0, self.cost_reference[int(j * ratio)] + random.normalvariate(0, self.cost_sigma))
                j += 1

            # Add the first cost for the day next
            cost[int(time + self._decision_time)] = self.cost_reference[0]

            for controller in self._controllers:         
                # Push the cost & Compute
                controller.cost = cost
                controller.outside_temperature_forecast = self.__temperature(
                    time,
                    time + self._decision_time + delta_time,
                    delta_time)

        #
        # Rest of the time
        #
        for controller in self._controllers:  
            controller.calculate(time, delta_time)

    def update(self, time, delta_time):
        super(AgregatorSimulator, self).update(time, delta_time)
        if time - delta_time in self.temperature_outside_forecast.keys():
            self.temperature = units.convert(units(self.temperature_outside_forecast[time - delta_time], units.degC), units.kelvin)
        else:
            # First step of the simulation
            self.outside_process.set_time(time)
            self.temperature = self.outside_process.temperature

    def __temperature(self, start, stop, delta_time):
        """
        This method compute a forecast for the temperature. It takes the real temperature and apply a variation to
        simulate a forecast with differences

        :param start: Starting time
        :type time

        :param stop: Stopping time
        :type time

        :param delta_time: Slot period time
        :type time

        :param common_unit: Unity for temperature
        :type time

        :return: A dictionary of forecasted temperature
        :rtype dictionary

        """

        if self.outside_process is not None:
            self.temperature_outside_forecast = {}

            # Day variance
            corr = random.normalvariate(0, self.day_sigma)

            for k in range(int(start), int(start+stop), int(delta_time)):
                self.outside_process.set_time(k * units.second)

                # Apply a variance with a normal distribution add to the real temperature
                corr += random.normalvariate(0, self.period_sigma)

                self.temperature_outside_forecast[k] = \
                    units.value(units.convert(getattr(self.outside_process, "temperature"), units.celsius)) + corr

            return self.temperature_outside_forecast
        else:
            return self.outside_temperature


class AgregatorElement(AbstractControllerElement):
    """
    The :class:`AgregatorElement` is an intermediate abstract class to receive
    the new cost and detect this receipt to recompute
    """
    def __init__(self, friendly_name, position=Position()):
        """
        Constructor for the abstract class AgregatorElement

        :param friendly_name: The friendly name

        :param position: Position of the element

        """
        super(AgregatorElement, self).__init__(friendly_name, position)
        self._cost = {}
        self._cost_has_changed = False

    @property
    def cost(self):
        """
        Return the vector of costs for each slot period

        :return: The vector of costs

        """
        return self._cost

    @cost.setter
    def cost(self, value):
        """
        Set the new cost if a change occur. The vector of cost is set by the central controller during
        the day.

        :param value: The vector of cost
        :type value: List

        """
        # Merge the correction
        self._cost = dict(self._cost.items() + value.items())
        self._cost_has_changed = True

    def total_cost(self):
        """
        Abstract method which returns the total of costs of the whole simulation

        :return: The total costs of the operation
        """
        raise NotImplementedError('Pure abstract method!')

    def total_power(self):
        """
        Abstract method which returns the total of power consumption of the whole simulation

        :return: The total power consumption of the operation
        """
        raise NotImplementedError('Pure abstract method!')

    def reset(self):
        """
        Abstract method inherited by `AbstractControllerElement`
        """
        raise NotImplementedError('Pure abstract method!')

    def calculate(self, time, delta_time):
        """
        Abstract method inherited by `AbstractControllerElement`
        """
        raise NotImplementedError('Pure abstract method!')

    def update(self, time, delta_time):
        """
        Abstract method inherited by `AbstractControllerElement`
        """
        raise NotImplementedError('Pure abstract method!')


class ForecastController(AgregatorElement):
    """
    This file implement the :class:`AgregatorSimulator` which is a module for gridsim. This represent a central
    controller which manage a set of local controllers implemented by :class:`AgregatorElement`.

    The central controller send information to the local controllers : the cost and the temperature forecast
    in degree for each slot duration.

    During the elapsing time, temperature could be different from the forecast. It leads the local controllers to
    difference of value. The local controllers can detect such an errors and recompute the decision. The central
    controllers could send new costs vector during the simulation, different from the forecast. In this case, the
    local controllers need to recompute their decision, too.

    The decision of an local controller is an on/off decision for each duration slot.

    The :class:`ElectroThermalHeaterCooler` represent a model of a heat pomp (heater, cooler, ...) made by
    Michael Clausen. It was used by the first version of Agreflex and is not present in Gridsim anymore. I import it
    for this demo.

    The :class:`AgregatorSimulator` is the main controller used as a gridsim module. The local controllers must be
    register to it. This module send weather and cost forecast once by period to the local controllers and correct in
    real time the cost if there is any change

    The :class:`AgregatorElement` is an intermediate abstract class to receive the new cost and detect this receipt to
    recompute

    The :class:`ForecastController` is a forecast controller. This class optimizes a thermal process given to a
    weather forecast and a cost vector.

    This class is the main implementation of the algorithms used to improve power balance and power efficiency.
    Regarding to a forecast cost and forecast weather, this local controller improve on/off operations. It computes
    the day-ahead optimization and correct computation in real time if a new cost or if the temperature forecast is
    too different of the real values. A decision is a switch on/switch off for each slot period.

    Decision operation:
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

    Error Management:
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

    """

    # The constants below is used for the error checking and correction of calculation

    MAX_TOTAL_ABSOLUTE_ERROR = 1.
    """
    The constant is the max absolute error computed. We use it to detect during the first computation day-ahead period
    if we need to consider errors.
    """

    MAX_TOTAL_ERROR = 0.10
    """
    Used to compute an error along the time. We can recompute the decision operation at any time if an error values
    is growing to fast.
    """

    MAX_REAL_TIME_ERROR = 0.5
    """
    Used to detect human behavior problem, forecast problem or setting configuration problem
    """

    MAX_DAY_HISTORIC = 10
    """
    Max days using for analysing a historic. It helps to detect error only after a certain time.
    """

    DELTA_TOLERANCE = 0.05
    """
    Used to detect if the error is due to a difference between forecast and real temperature
    """

    class Correction(object):
        """
        This inner class keep the current corrections settings.
        """
        def __init__(self, thermal_conductivity, temperature):
            self.thermal_conductivity = thermal_conductivity
            self.temperature = temperature

    def __init__(self, friendly_name, target_temperature, hysteresis, thermal_process, subject, attribute,
                 decision_time, delta_time, solver=None, on_value=True, off_value=False, position=Position()):

        """
        A forecast controller. This class optimizes a thermal process given to a weather forecast and
        a cost vector.

        :param: friendly_name: User friendly name to give to the element.
        :type friendly_name: str, unicode

        :param: target_temperature: The temperature to try to maintain inside the target ThermalProcess.
        :type: target_temperature: float, int

        :param: hysteresis: The +- hysteresis in order to keep in an area of variation of temperature
        :type: hysteresis: float, int

        :param: thermal_process: The reference to the thermal process to observe.
        :type: thermal_process: ThermalProcess

        :param: subject: Reference to the object of which's attribute has to be changed depending on the termperature.
        :type: object

        :param: attribute: The name of the attribute to control as string.
        :type: str, unicode

        :param decision_time: Step of decision
        :type decision_time: float

        :param delta_time: Time interval for the simulation in seconds.
        :type delta_time: float

        :param: on_value: The value to set for the attribute in order to turn the device "on".
        :type: on_value: Boolean

        :param: off_value: The value to set for the attribute in order to turn the device "off".
        :type: off_value: Boolean

        :param position: The position of the thermal element. Defaults to [0,0,0].
        :type position: :class:`Position`

        """

        super(ForecastController, self).__init__(friendly_name, position)

        if not isinstance(target_temperature, units.Quantity):
            raise TypeError('target_temperature')
        self.target_temperature = units.value(target_temperature)
        """
        The temperature to try to retain inside the observer thermal process by conducting an electrothermal element.
        """

        if not isinstance(hysteresis, (float, int)):
            raise TypeError('hysteresis')
        self.hysteresis = hysteresis
        """
        The +- hysteresis applied to the temperature measure in order to avoid to fast on/off switching.
        """

        if not isinstance(thermal_process, AbstractSimulationElement) \
                and not hasattr(thermal_process, 'temperature') \
                and not hasattr(thermal_process, 'thermal_volumic_capacity'):
            raise TypeError('thermal_process')
        self.thermal_process = thermal_process
        """
        The reference to the thermal process to observe and read the temperature from.
        """

        if not isinstance(subject, ElectroThermalHeaterCooler) and not hasattr(subject, 'power'):
            raise TypeError('subject')
        self.subject = subject
        """
        The reference to the element to control.
        """

        if not isinstance(attribute, (str, unicode)):
            raise TypeError('attribute')
        self.attribute = attribute
        """
        Name of the attribute to control.
        """

        self.on_value = on_value
        """
        Value to set in order to turn the element on.
        """

        self.off_value = off_value
        """
        Value to set in order to turn the element off.
        """

        self.decision_time = int(units.value(units.convert(decision_time, units.second)))
        """
        Value of decision time. The optimization will be computed each step
        """

        self.delta_time = int(units.value(units.convert(delta_time, units.second)))
        """
        Value of the unit of time used to decide the optimized comsumption
        """

        # Used for detecting errors
        self.error = 0
        self.count_error = 0
        self.absolute_error = 0
        self.total_error = 0
        self.total_absolute_error = 0

        # Used for computing
        self.countOptimization = 0
        self.mean = target_temperature
        self.temperature_optimal = target_temperature
        self.old_temperature = target_temperature

        self._outside_temperature_forecast = {}

        # Used for historic, statistics, plotting, ...
        self._instant_cost = 0  # for plotting
        self._total_cost = 0
        self._total_power = 0
        self._output_value = off_value
        self._power_on = {0: 0}
        self._history_temperature = []
        self._historic_for_correction = []
        self._current_correction = None  # Correction if room next
        self._current_temperature_correction = 0.
        self._outside_process = None
        self._outside_coupling = None
        # self._external_process = []  # not used yet
        # self._external_coupling = []  # not used yet

        # Cannot use pulp.GUROBI as default value parameter because msg=0 set false for the gurobi solver statically
        if solver is None:
            self.solver = pulp.GLPK(msg=0)
        else:
            self.solver = solver

    @property
    def outside_temperature_forecast(self):
        """
        Getter for the temperature forecast
        :return: A dictionary for the temperature forecast
        """

        return self._outside_temperature_forecast

    @outside_temperature_forecast.setter
    def outside_temperature_forecast(self, value):
        """
        Setter for the new temperature forecast. The temperature correction is re-init.
        :param value: A dictionary of temperature forecast
        """
        self._current_temperature_correction = 0.
        self._outside_temperature_forecast = value

    def reset(self):
        """
        AbstractSimulationElement implementation, see :func:`agreflex.core.AbstractSimulationElement.reset`.
        """
        pass

    def calculate(self, time, delta_time):
        """
        Calculation step. We compute the schedule for the day during the day-ahead time. This is the optimization
        process / decision operation.
        """

        # Update the cost vector regarding to the time
        # We reduce this vector if we need to recompute during real time. We consider only the remaining slots period
        self._cost = {k: v for k, v in self._cost.items() if k >= time}

        # Compute decision for each delta_time
        # If decision time or if cost has changed
        # We can't update the actual state. So we take decision for the next step.
        if self._cost_has_changed:
            self._recompute_decision_operation_due_to_new_costs(time)

        # Recompute in real time ?
        # Check if we need to recompute the optimization due to a difference of forecast
        # Compare forecast temperature with real temperature and recompute if the difference is to high.
        # Recompute if sufficient of historic
        if time - self.delta_time in self.temperature_optimal.keys():
            self._detect_if_error_and_recompute_if_necessary(time)

        # Historic & Stats
        self.old_temperature = units.value(units.convert(units(self.thermal_process.temperature, units.kelvin), units.celsius))
        self._history_temperature.append(self.old_temperature)
        self._instant_cost = self._cost[time]
        self._total_cost += self._cost[time] * self._power_on[time]
        self._total_power += self._power_on[time] * self.subject.power

        # Decision for the next step
        self._output_value = self.on_value if int(self._power_on[time + delta_time]) == 1 else self.off_value

    def update(self, time, delta_time):
        """
        Update default method
        """
        setattr(self.subject, self.attribute, self._output_value)

    def add(self, thermal_process, thermal_coupling):
        """
        Allow to add a thermal process with its thermal coupling
        :param thermal_process: The thermal process
        :param thermal_coupling: The thermal coupling associate with the thermal process
        """
        if not isinstance(thermal_coupling, ThermalCoupling) or thermal_coupling is None:
            raise RuntimeError('Missing or invalid thermalCoupling reference.')

        if thermal_process is not None:
            if isinstance(thermal_process, TimeSeriesThermalProcess):
                self._outside_process = thermal_process
                self._outside_coupling = thermal_coupling
                return
            else:
                raise RuntimeError('Can use only TimeSeriesThermalProcess')
#            if isinstance(thermal_process, ThermalProcess):
#                self._external_process.append(thermal_process)
#                self._external_coupling.append(thermal_coupling)
#                return

        raise RuntimeError('Missing or invalid thermalProcess or thermalCoupling reference.')

    def total_cost(self):
        """
        Return the sum of costs according to the whole simulation
        :return: The sum of costs
        """
        return self._total_cost

    def total_power(self):
        """
        Return the sum of power consumption according to the whole simulation

        :return: The sum of power consumption
        """
        return self._total_power

    def __optimize(self, cost, first_decision=0, correction=None, delta_temperature_outside_correction=0.):
        """
        Optimization process. Can be use during the day-ahead decision period or in real time if errors. Set a
        on/off value for each slot.

        :param cost: Cost using for the minimization function
        :type cost: Vector

        :param first_decision: We can't decide for the current time. We need the first decision of the simulation

        :param correction: Used if errors has been detected in the past
        :type correction: Correction

        :param delta_temperature_outside_correction: Used if difference of temperature has been detected. The difference
                                                     is the delta between the forecase and the real value.
        :type delta_temperature_outside_correction: float

        :return A tuple of two dictionaries: The first dictionary : Key is the time and value is the on/off
                                             value decision. The second dictionary : Key is the time  t and value is the
                                             final temperature at time t.

        """

        ###########################################################
        #
        # Set the data
        #

        starting_time = sorted(cost.keys())[0]
        ending_time = sorted(cost.keys())[-1] + self.delta_time

        # Value importation
        # external_thermal_element = self._outside_process # Get the temperature at time t
        thermal_conductivity = units.value(getattr(self._outside_coupling, "thermal_conductivity"))  # with couple
        thermal_capacity = units.value(self.thermal_process.thermal_volumic_capacity)
        if thermal_capacity == 0:
            raise RuntimeError("Thermal capacity must be greater than zero")

        subject_energy = units.value(self.subject.power)
        subject_efficiency = getattr(self.subject, "_efficiency_factor")

        nb_slots_optimization = len(cost)

        #
        # Set the data
        #
        ###########################################################

        ###########################################################
        #
        # Prepare the problem
        #

        # Linearization of the problem
        problem = pulp.LpProblem("lpOptimization", pulp.LpMinimize)
        # Decisions variables
        initial_temperature = {}
        final_temperature = {}
        _power_on = {}
        exceeding = {}
        difference_mean_temperature = pulp.LpVariable("diff_mean", lowBound=0)

        # Data
        temperature = {}

        # For each step of optimisation
        for t in range(starting_time, ending_time, self.delta_time):
            # Prepare the variables
            initial_temperature[t] = pulp.LpVariable("tinit_%s" % t)
            final_temperature[t] = pulp.LpVariable("tfin_%s" % t)
            _power_on[t] = pulp.LpVariable("p_%s" % t, cat='Binary')
            exceeding[t] = pulp.LpVariable("exc_%s" % t, lowBound=0)
            temperature[t] = \
                units.value(
                    units.convert(self._outside_temperature_forecast[t], units.celsius)
                ) + delta_temperature_outside_correction

        # Objective to minimize the cost
        problem += \
            pulp.lpSum(
                _power_on[t] * cost[t] + 1000 * exceeding[t] for t in cost.keys()
            ) + 10000 * difference_mean_temperature

        #
        # Constraints
        #
        # Temperature mean during the time optimization must be the
        # target temperature (greater than because lost precision float)
        problem += \
            pulp.lpSum(final_temperature[t] for t in final_temperature.keys()) >= \
            units.value(
                units.convert(self.target_temperature, units.celsius)
            ) * nb_slots_optimization - difference_mean_temperature

        if correction is None:
            for t in range(starting_time, ending_time, self.delta_time):

                outside_leverage = self.__external_leverage(
                    thermal_capacity=thermal_capacity,
                    thermal_conductivity=thermal_conductivity,
                    external_temperature=temperature[t],
                    internal_temperature=initial_temperature[t])

                inner_production = self.__internal_production(
                    internal_temperature=initial_temperature[t],
                    power_on=_power_on[t],
                    subject_efficiency=subject_efficiency,
                    subject_energy=subject_energy,
                    thermal_capacity=thermal_capacity)

                problem += final_temperature[t] == outside_leverage + inner_production

        else:
            for t in range(starting_time, ending_time, self.delta_time):
                outside_leverage = self.__external_leverage(
                    thermal_capacity=thermal_capacity,
                    thermal_conductivity=thermal_conductivity,
                    external_temperature=temperature[t],
                    internal_temperature=initial_temperature[t])

                inner_production = self.__internal_production(
                    internal_temperature=initial_temperature[t],
                    power_on=_power_on[t],
                    subject_efficiency=subject_efficiency,
                    subject_energy=subject_energy,
                    thermal_capacity=thermal_capacity)

                correction_leverage = self.__external_leverage(
                    thermal_capacity=thermal_capacity,
                    thermal_conductivity=correction.thermal_conductivity,
                    external_temperature=correction.temperature,
                    internal_temperature=initial_temperature[t])

                problem += final_temperature[t] == outside_leverage + inner_production + correction_leverage

        for t in range(starting_time, ending_time, self.delta_time):
            problem += final_temperature[t] <= self.target_temperature + 0.5 * self.hysteresis + exceeding[t]
            problem += final_temperature[t] >= self.target_temperature - 0.5 * self.hysteresis - exceeding[t]
        problem += \
            initial_temperature[starting_time] == \
            units.value(units.convert(units(self.thermal_process.temperature, units.kelvin), units.celsius))

        problem += _power_on[starting_time] == first_decision

        for t in range(starting_time + self.delta_time, ending_time, self.delta_time):
            problem += initial_temperature[t] == final_temperature[t-self.delta_time]

        #
        # Prepare the problem
        #
        ###########################################################

        ###########################################################
        #
        # Resolution and return
        #

        problem.solve(self.solver)
        self.status = pulp.LpStatus[problem.status]

        if self.status is 'Infeasible':
            raise RuntimeError("The problem isn't feasible")

        return [
            dict([(k, pulp.value(v)) for k, v in _power_on.items()]),
            dict([(k, pulp.value(v)) for k, v in final_temperature.items()])
        ]

    def __is_delta_temperature_error(self):
        """
        Detect if there is an error due to a difference between the forecast and the real temperature
        :return: If there is errors due to a difference of temperature
        """

        ###############################################################################################################
        #
        # Load & prepare data
        #

        subject_efficiency = getattr(self.subject, "_efficiency_factor")
        subject_thermal_capacity = units.value(self.thermal_process.thermal_volumic_capacity)
        subject_thermal_outside_coupling = units.value(self._outside_coupling.thermal_conductivity)

        temperature = [self._historic_for_correction[0][1]]
        error = 0.

        #
        # Load & prepare data
        #
        ###############################################################################################################

        ###############################################################################################################
        #
        # Compute difference of temperature regarding to the real temperature in contrast to the forecast temperature
        #
        for time, \
            init_temp,\
            opt_temp, \
            found_temp,\
            temperature_ext_forecast,\
            temperature_ext,\
            power \
                in self._historic_for_correction:

            last_temperature = temperature[-1]

            outside_leverage = self.__external_leverage(
                thermal_capacity=subject_thermal_capacity,
                thermal_conductivity=subject_thermal_outside_coupling,
                external_temperature=temperature_ext,
                internal_temperature=last_temperature)

            inner_production = self.__internal_production(
                internal_temperature=init_temp,
                power_on=power,
                subject_efficiency=subject_efficiency,
                subject_energy=units.value(self.subject.power),
                thermal_capacity=subject_thermal_capacity)

            correction_leverage = 0
            if self._current_correction is not None:
                correction_leverage = self.__external_leverage(
                    thermal_capacity=subject_thermal_capacity,
                    thermal_conductivity=self._current_correction.thermal_conductivity,
                    external_temperature=units.value(self._current_correction.temperature),
                    internal_temperature=last_temperature)

            final_temperature = outside_leverage + inner_production + correction_leverage

            error += found_temp - final_temperature

            temperature.append(final_temperature)

        debug("| | + {} {}".format(abs(error), ForecastController.DELTA_TOLERANCE * len(self._historic_for_correction)))
        return abs(error) <= ForecastController.DELTA_TOLERANCE * len(self._historic_for_correction)

        #
        # Compute difference of temperature regarding to the real temperature in contrast to the forecast temperature
        #
        ###############################################################################################################

    def __check_correction(self):
        """
        If the historic is big enough, we can try to find the correction value if a adjacency room is set by the user
        We use genetic algorithms to find two settings: thermal conductivity and the temperature of the adjacency room
        :return: A tuple. First element is the correction. Second element is a boolean which precise if a better
        correction as the current one has been found.
        """

        debug("| | + Check correction {}".format(len(self._historic_for_correction)))
        if len(self._historic_for_correction) == 0:
            return self._current_correction, False

        #
        # Check if the error is due to difference between real and forecasted temperature
        #
        if self.__is_delta_temperature_error():
            debug("| | | + Temperature error")
            return self._current_correction, False

        ###############################################################################################################
        #
        # Load data & prepare genetic components
        #

        subject_efficiency = getattr(self.subject, "_efficiency_factor")
        subject_thermal_capacity = units.value(self.thermal_process._thermal_capacity)
        subject_thermal_outside_coupling = units.value(self._outside_coupling.thermal_conductivity)

        # Create individual with his attributes
        creator.create("FitnessMinError", base.Fitness, weights=(-1.0,))
        creator.create("Individual", list, fitness=creator.FitnessMinError)

        toolbox = base.Toolbox()

        # Attribute generator
        toolbox.register("thermal_conductivity", random.randint, -1000., 1000.)
        toolbox.register("temperature", random.randint, -50, 50)

        # Structure initializer
        toolbox.register("individual", tools.initCycle, creator.Individual,
                         (toolbox.thermal_conductivity, toolbox.temperature), n=1)
        toolbox.register("population", tools.initRepeat, list, toolbox.individual)

        #
        # Load data & prepare genetic components
        #
        ################################################################################################################

        ################################################################################################################
        #
        # Fitness evaluation & mutation operator
        #
        def evaluate(individual):

            weight = 0.0

            individual_thermal_conductivity = individual[0]
            individual_constant_temperature = individual[1]

            eval_temperature = [self._historic_for_correction[0][1]]

            for eval_time, \
                eval_init_time,\
                eval_opt_temp,\
                eval_found_temp,\
                eval_temperature_ext_forecast,\
                evaL_temperature_ext,\
                eval_power \
                    in self._historic_for_correction:

                eval_last_temperature = eval_temperature[-1]

                eval_outside_leverage = self.__external_leverage(
                    thermal_capacity=subject_thermal_capacity,
                    thermal_conductivity=subject_thermal_outside_coupling,
                    external_temperature=evaL_temperature_ext + self._current_temperature_correction,
                    internal_temperature=eval_last_temperature)

                eval_inner_production = self.__internal_production(
                    internal_temperature=eval_init_time,
                    power_on=eval_power,
                    subject_efficiency=subject_efficiency,
                    subject_energy=units.value(self.subject.power),
                    thermal_capacity=subject_thermal_capacity)

                if self._current_correction is None:
                    correction_leverage = self.__external_leverage(
                        thermal_capacity=subject_thermal_capacity,
                        thermal_conductivity=individual_thermal_conductivity,
                        external_temperature=individual_constant_temperature,
                        internal_temperature=eval_last_temperature)
                else:
                    correction_leverage = self.__external_leverage(
                        thermal_capacity=subject_thermal_capacity,
                        thermal_conductivity=individual_thermal_conductivity
                        + self._current_correction.thermal_conductivity,
                        external_temperature=individual_constant_temperature+self._current_correction.temperature,
                        internal_temperature=eval_last_temperature)

                eval_final_temperature = eval_outside_leverage + eval_inner_production + correction_leverage

                eval_temperature.append(eval_final_temperature)

                weight += abs(eval_found_temp - eval_final_temperature) * 10000

            return weight,

        def mut_set(individual):

            individual[0] += random.uniform(-10, 10)  # thermal_coupling
            individual[1] += random.uniform(-5, 5)  # temperature

            return individual,

        #
        # Fitness evaluation & mutation operator
        #
        ################################################################################################################

        ################################################################################################################
        #
        # Initialize & compute
        #

        toolbox.register("evaluate", evaluate)
        # toolbox.register("mate", cxSet)
        toolbox.register("mate", tools.cxUniform, indpb=0.5)
        # toolbox.register("mate", tools.cxOnePoint)
        toolbox.register("mutate", mut_set)
        toolbox.register("select", tools.selTournament, tournsize=3)
        # toolbox.register("select", tools.selNSGA2)

        nb_generation = 60  # 60 or 200
        nb_population = 300  # 300 or 500
        crossing_rate = 0.3  # 0.5
        mutation_rate = 0.4  # 0.6

        pop = toolbox.population(n=nb_population)
        hof = tools.ParetoFront()
        stats = tools.Statistics(lambda ind: ind.fitness.values)
        stats.register("avg", numpy.mean, axis=0)
        stats.register("std", numpy.std, axis=0)
        stats.register("min", numpy.min, axis=0)
        stats.register("max", numpy.max, axis=0)

        pop, _ = algorithms.eaSimple(
            pop,
            toolbox,
            stats=stats,
            cxpb=crossing_rate,
            mutpb=mutation_rate,
            ngen=nb_generation,
            halloffame=hof,
            verbose=False
        )
        best = tools.selBest(pop, 1)[0]
        score = best.fitness.values[0]

        #
        # Initialize & compute
        #
        ################################################################################################################

        ################################################################################################################
        #
        # Compute the actual score and compare it with the new
        # If the new is better, we swap the current correction
        #
        temperature = [self._historic_for_correction[0][1]]
        old_score = 0.
        score_basic = 0.
        for time, init_temp,\
            opt_temp,\
            found_temp, \
            temperature_ext_forecast,\
            temperature_ext,\
            power \
                in self._historic_for_correction:

            old_score += abs(opt_temp - found_temp) * 10000

            last_temperature = temperature[-1]
            outside_leverage = self.__external_leverage(
                thermal_capacity=subject_thermal_capacity,
                thermal_conductivity=subject_thermal_outside_coupling,
                external_temperature=temperature_ext,
                internal_temperature=last_temperature)

            inner_production = self.__internal_production(
                internal_temperature=init_temp,
                power_on=power,
                subject_efficiency=subject_efficiency,
                subject_energy=units.value(self.subject.power),
                thermal_capacity=subject_thermal_capacity)

            final_temperature = outside_leverage + inner_production
            temperature.append(final_temperature)
            score_basic += abs(found_temp - final_temperature) * 10000

        if score_basic < old_score and score_basic < score:
            debug("| | | + reinit")
            return None, True

        if score > old_score or score == float('nan'):
            debug("| | | + keep current correction")
            return self._current_correction, False

        if self._current_correction is None:
            c = self.Correction(best[0], best[1])
            debug("| | | + swap with correction: {} and {} fitness value [{} {}]".format(best,
                                                                                         best.fitness.values[0],
                                                                                         c.thermal_conductivity,
                                                                                         c.temperature))
            return c, True

        c = self.Correction(
            best[0] + self._current_correction.thermal_conductivity,
            best[1] + self._current_correction.temperature
        )

        return c, True

        #
        # Compute the actual score and compare it with the new
        #
        ################################################################################################################

    def __external_leverage(self, thermal_capacity, thermal_conductivity, external_temperature, internal_temperature):
        """
        Used to compute the external (or outside) leverage. Return the inside temperature according to the parameters
        :param thermal_capacity: Thermal capacity of the room
        :param thermal_conductivity: Thermal conductivity with outside
        :param external_temperature: The outside temperature
        :param internal_temperature: The room temperature

        :return: The inside temperature according all the parameters
        """
        return (self.delta_time / thermal_capacity) \
            * (thermal_conductivity * (external_temperature - internal_temperature))

    def __internal_production(self,
                              internal_temperature,
                              power_on,
                              subject_efficiency,
                              subject_energy,
                              thermal_capacity):
        """
        Compute the temperature according to the heater/cooler decision
        :param internal_temperature: Room temperature
        :param power_on: Switch on/off the heater/cooler
        :param subject_efficiency: Efficiency of the heat pomp
        :param subject_energy: Energy of the heat pomp
        :param thermal_capacity: Thermal capacity of the room

        :return: Temperature according to the switch on/off decision
        """

        return internal_temperature + power_on * subject_efficiency \
            * ((subject_energy / thermal_capacity) * self.delta_time)

    def __recompute_with_temperature_error(self, time):
        """
        Method used to recompute the decision operation if an forecast error is detected
        :param time: Time
        """

        if len(self._history_temperature) > 0:
            debug("| | + due to temperature error : {}".format(numpy.mean(self._history_temperature[-20:])))
        else:
            debug("| | + due to temperature error")

        self._current_temperature_correction = \
            units.value(
                units.convert(
                    getattr(self._outside_process, "temperature"),
                    units.celsius
                )
            ) - self._outside_temperature_forecast[time]

        self._power_on, self.temperature_optimal = self.__optimize(
            cost=self._cost,
            first_decision=self._power_on[time],
            correction=self._current_correction,
            delta_temperature_outside_correction=self._current_temperature_correction)

        debug("| | |- Correction temperature {} {} {}".format(
            units.value(
                units.convert(
                    getattr(
                        self._outside_process,
                        "temperature"
                    ),
                    units.celsius
                )
            ) - self._outside_temperature_forecast[time],
            units.value(units.convert(getattr(self._outside_process, "temperature"), units.celsius)),
            self._outside_temperature_forecast[time]))

    def _recompute_decision_operation_due_to_new_costs(self, time):
        """
        Re-compute the decision operation if a new costs vector has been sent

        :param time: Time of computation
        """

        self._cost_has_changed = False

        debug("+ Decision time {}".format(int(time / self.decision_time)))

        # Maximum time for the current or future optimization
        ForecastController.target_time = int(time + self.decision_time + 1 * self.delta_time)

        nb_slots_optimization = len(self._cost)

        # Conditions to minimize errors correction
        if self.count_error > 0:
            admissible_error = self.total_absolute_error / self.count_error

            #
            # Absolute error is too big
            #
            if admissible_error > ForecastController.MAX_TOTAL_ABSOLUTE_ERROR / self.delta_time:
                debug("| + absolute error override")
                self._current_correction, fountBetterCorrection = self.__check_correction()
                if fountBetterCorrection:
                    self._historic_for_correction = []

            #
            # Need a significant historic to recompute
            #
            if admissible_error > ForecastController.MAX_TOTAL_ABSOLUTE_ERROR \
                    / (self.delta_time * ForecastController.MAX_DAY_HISTORIC) \
                    and len(self._historic_for_correction) \
                    > (ForecastController.MAX_DAY_HISTORIC * nb_slots_optimization):

                debug("| + full historic")
                self._current_correction, fountBetterCorrection = self.__check_correction()
                self._historic_for_correction = []

            self.total_absolute_error = 0.0
            self.total_error = 0.0
            self.count_error = 0

        #
        # Optimize process
        #
        if int(time) not in self._power_on.keys():
            self._power_on[int(time)] = 0

        self._power_on, self.temperature_optimal = self.__optimize(
            cost=self._cost,
            first_decision=self._power_on[int(time)],
            correction=self._current_correction)

        debug("| + classic optimized")

        #
        # Statistics and historic
        #
        self.countOptimization += 1
        if len(self._history_temperature) > 0:
            self._history_temperature = self._history_temperature[-96:]
            self.mean = units(numpy.mean(self._history_temperature), units.degC)
            debug("| | + Mean: {}".format(self.mean))

    def _detect_if_error_and_recompute_if_necessary(self, time):
        """
        Detect if an error is detected. If the error is too big, we recompute the decision operation

        :param time: The time of operation
        """

        #
        # Compute the error
        # difference between optimal temperature computed and real temperature
        #
        self.error = \
            self.temperature_optimal[time - self.delta_time] - \
            units.value(units.convert(units(self.thermal_process.temperature, units.kelvin), units.celsius))
        self.error = self.error if abs(self.error) >= 1e-10 else 0.0
        self.absolute_error = abs(self.error)
        self.total_error += self.error
        self.total_absolute_error += abs(self.error)
        self.count_error += 1

        #
        # Historic
        #
        self._outside_process.set_time(time * units.second)
        self._historic_for_correction.append((
            time,
            self.old_temperature,
            self.temperature_optimal[time - self.delta_time],
            units.value(units.convert(units(self.thermal_process.temperature, units.kelvin), units.celsius)),
            self._outside_temperature_forecast[time],
            units.value(units.convert(self._outside_process.temperature, units.celsius)),
            self._power_on[time - self.delta_time]))
        #
        # Determine if we need to consider the error
        #
        if self.absolute_error >= ForecastController.MAX_REAL_TIME_ERROR \
                and self.absolute_error > 2 * abs(self.total_error / self.count_error):

            debug("| + current error too big {}".format(self.error))
            if self.__is_delta_temperature_error():

                #
                # Recompute according to the difference of temperature regarding to the forecast
                #
                self.__recompute_with_temperature_error(time)
                self._historic_for_correction = []

            else:
                #
                # Error due to a human behavior, must recompute
                #
                self._power_on, self.temperature_optimal = self.__optimize(
                    cost=self._cost,
                    first_decision=self._power_on[int(time)],
                    correction=self._current_correction)

                self.countOptimization += 1
                self._historic_for_correction = []

            self.total_absolute_error = 0.0
            self.total_error = 0.0
            self.count_error = 0

        elif abs(self.total_error) / self.count_error >= ForecastController.MAX_TOTAL_ERROR \
                and len(self._historic_for_correction) >= ForecastController.MAX_DAY_HISTORIC:

            debug("| + total error too big {}".format(self.total_error))

            if self.__is_delta_temperature_error():

                #
                # Recompute according to the difference of temperature
                #
                self.__recompute_with_temperature_error(time)
                self._historic_for_correction = []

            else:
                #
                # Compute correction due to adjacency room
                # Use only in the case of placing an adjacency room with a coupling to the main room
                #
                self._current_correction, fountBetterCorrection = self.__check_correction()
                if fountBetterCorrection:
                    #
                    # Re-optimize with corrections
                    #
                    self._historic_for_correction = []
                    self._power_on, self.temperature_optimal = self.__optimize(
                        cost=self._cost,
                        correction=self._current_correction)
                    self.countOptimization += 1

            self.total_absolute_error = 0.0
            self.total_error = 0.0
            self.count_error = 0

            #
            # Recompute in real time ?
            ###############################################################################################################


Simulator.register_simulation_module(AgregatorSimulator)
