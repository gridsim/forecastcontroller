# -*- coding: utf-8 -*-
"""
.. codeauthor:: Joel Cavat <joel.cavat@hesge.ch>

This demo can be run from the demo folder entering "python forecastcontroller/forecastcontroller.py" inside
the console.

This demo use an agregator as a main controller which pilot some forecastcontroller attached to a
ElectroThemralHeaterCooler device.

The main controller send a forecast each decision time regarding the meteo and the cost for the next period.
Each local controller compute a forecast of on/off running decision for each step time. Then, during the simulation,
if the temperature change or if the cost change, the main controller can send again these information to each
local forecastcontroller. They can readjust their on/off running decision.


"""
import sys
import os
import random
import json
import pulp

sys.path.append(os.path.abspath("../"))

from gridsim.unit import units
from gridsim.simulation import Simulator
from gridsim.recorder import PlotRecorder
from gridsim.thermal.element import TimeSeriesThermalProcess
from gridsim.thermal.core import ThermalProcess, ThermalCoupling
from gridsim.electrical.network import ElectricalPQBus, \
    ElectricalTransmissionLine
from gridsim.electrical.loadflow import DirectLoadFlowCalculator
from gridsim.timeseries import SortedConstantStepTimeSeriesObject
from gridsim.iodata.input import CSVReader
from gridsim.iodata.output import FigureSaver

from agregator import ForecastController, ElectroThermalHeaterCooler


# Gridsim TimeSeriesThermalProcess doesn't provide set_time anymore. Agreflex version provided it.
class TimeSeriesThermalProcessAgreflexCompatible(TimeSeriesThermalProcess):
    def set_time(self, time):
        self._time_series.set_time(time)
        self.temperature = self._time_series.temperature


class ConfigException(Exception):
    def __init__(self, parameter, message=None):
        self.parameter = parameter
        self.message = message

    def __str__(self):
        message = "************************\n"
        message += "ConfigException raised:\n"
        message += "> Incompatible value for parameter(s) {}".format(self.parameter)
        if self.message is not None:
            message += ", {}.".format(self.message)
        message += "\n************************"
        return message


if __name__ == "__main__":

    sub_path = None
    if len(sys.argv) > 1:
        sub_path = sys.argv[1]

    # Create simulator.
    sim = Simulator()
    sim.electrical.load_flow_calculator = DirectLoadFlowCalculator()

    ####################################################################################
    # Basic configuration :
    #
    with open("forecastcontroller/config.json") as json_config_file:
        json_config = json.load(json_config_file)
        START_TIME = 0
        DURATION_TIME = json_config["simulation_configuration"]["duration_day"] * units.day  # 1 * units.year
        DECISION_DURATION_SLOT = json_config["simulation_configuration"]["decision_duration_day_period"] * units.day
        DURATION_MINUTE_SLOT = json_config["simulation_configuration"]["duration_minute_slot"] * units.minute

        try:
            # Buildings parameters
            ROOM_SIZE_MIN = json_config["device_configuration"]["size_min_meter"]
            ROOM_SIZE_MAX = json_config["device_configuration"]["size_max_meter"]
            if not ROOM_SIZE_MAX >= ROOM_SIZE_MIN >= 0:
                raise ConfigException("ROOM_SIZE_MIN or ROOM_SIZE_MAX")

            HEIGHT_MIN = json_config["device_configuration"]["height_min_meter"]
            HEIGHT_MAX = json_config["device_configuration"]["height_max_meter"]
            if not HEIGHT_MAX >= HEIGHT_MIN >= 0:
                raise ConfigException("HEIGHT_MIN or HEIGHT_MAX", "Incompatible values")

            POWER_MIN = json_config["device_configuration"]["power_min_watt"]
            POWER_MAX = json_config["device_configuration"]["power_max_watt"]
            if not POWER_MAX >= POWER_MIN >= 0:
                raise ConfigException("POWER_MIN or POWER_MAX")

            HYSTERESIS_MIN = json_config["device_configuration"]["hysteresis_user_preference_min"]
            HYSTERESIS_MAX = json_config["device_configuration"]["hysteresis_user_preference_max"]
            if not HYSTERESIS_MAX >= HYSTERESIS_MIN >= 0:
                raise ConfigException("HYSTERESIS_MIN or HYSTERESIS_MAX")

            # Nb buildings with devices and controllers
            MAX_INDEX_NB_DEVICES = json_config["device_configuration"]["number"]
            if not MAX_INDEX_NB_DEVICES >= 0:
                raise ConfigException("MAX_INDEX_NB_DEVICES", "The value must be greater or equal than zero")

            # Example of costs
            COST = json_config["simulation_configuration"]["cost_period_slot"]

            if not units\
                .value(
                    units.convert(DECISION_DURATION_SLOT, units.unit(DURATION_MINUTE_SLOT))
                    ) / units.value(DURATION_MINUTE_SLOT) == len(COST):
                print "WARNING: the cost_period_slot ratio is not equal to the number for daily slot."

            COUPLING_VARIANCE_MIN = json_config["device_configuration"]["coupling_min"]
            if not COUPLING_VARIANCE_MIN >= 0:
                raise ConfigException("COUPLING_VARIANCE_MIN", "Coupling could be less than zero with this value")

            COUPLING_VARIANCE_MAX = json_config["device_configuration"]["coupling_max"]
            if not COUPLING_VARIANCE_MAX >= COUPLING_VARIANCE_MIN:
                raise ConfigException("COUPLING_VARIANCE_MAX")

            TEMPERATURE_MIN = json_config["device_configuration"]["temperature_min"]
            if not TEMPERATURE_MIN >= -273.15:
                raise ConfigException("TEMPERATURE_MIN", "Temperature min must be greater or equal than -273.15 degrees")

            TEMPERATURE_MAX = json_config["device_configuration"]["temperature_max"]
            if not TEMPERATURE_MAX > TEMPERATURE_MIN:
                raise ConfigException("TEMPERATURE_MAX", "Temperature max must be greater than temperature min")

            DAY_SIGMA = json_config["simulation_configuration"]["weather_degree_day_sigma"]
            SLOT_SIGMA = json_config["simulation_configuration"]["weather_degree_slot_sigma"]
            COST_SIGMA = json_config["simulation_configuration"]["cost_sigma"]
            if not DAY_SIGMA >= 0 or not SLOT_SIGMA >= 0 or not COST_SIGMA >= 0:
                raise ConfigException(
                    "DAY_SIGMA, SLOT_SIGMA or COST_SIGMA",
                    "Value must be greater or equal than zero")

            message = 1 if json_config["solver_configuration"]["messages"] == "true" else 0
            timeout = json_config["solver_configuration"]["time_limit_second"]
            timeout = timeout if timeout > 0 else None

            solvers_list = {
                "gurobi": pulp.GUROBI(msg=message, timeLimit=timeout),
                "cplex": pulp.CPLEX_DLL(msg=message, timeLimit=timeout),
                "cbc": pulp.COIN(msg=message, maxSeconds=timeout),
                "glpk":
                    pulp.GLPK(msg=message,
                              options=['--tmlim', str(timeout)]) if timeout is not None else pulp.GLPK(msg=message)
            }
            SOLVER = None
            solvers = json_config["solver_configuration"]["solvers"]
            while solvers:
                solver_txt = solvers[0]
                if solver_txt in solvers_list.keys():
                    solver = solvers_list[solver_txt]
                    if solver.available():
                        SOLVER = solver
                        solvers = []

                solvers = solvers[1:]

            print "solver is {}".format(SOLVER)

            if SOLVER is None:
                raise ConfigException("SOLVER", "No solver available")

        except ConfigException as cexp:
            print cexp
            sys.exit(0)

    # Outside thermal element
    outside = sim.thermal.add(
        TimeSeriesThermalProcessAgreflexCompatible(
            'Outside Temperature',
            SortedConstantStepTimeSeriesObject(CSVReader()),
            './data/example_time_series.csv',
            lambda t: t*units.hour,
            temperature_calculator=lambda t: units.convert(units(t, units.degC), units.kelvin)))

    # This list is used to register and graph some information about temperature and consumptions
    lstTemperatureMonitoring = [outside]

    # Create a minimal electrical simulation network with a thermal heater connected to Bus0.
    bus0 = sim.electrical.add(ElectricalPQBus('Bus0'))
    sim.electrical.connect("Line0", sim.electrical.bus(0), bus0, ElectricalTransmissionLine('Line0',
                                                                                            1000*units.meter,
                                                                                            0.2*units.ohm))

    # Lists used for the graph
    lstForecast = []
    lstHeater = []

    # Creation of a set of building, their devices and their forecastcontroller and their own parameter
    for i in range(1, MAX_INDEX_NB_DEVICES + 1):

        # Random parameters for buildings
        size = random.randint(ROOM_SIZE_MIN, ROOM_SIZE_MAX)
        power = random.randint(POWER_MIN, POWER_MAX)
        hysteresis = random.randint(HYSTERESIS_MIN, HYSTERESIS_MAX)
        coupling = random.randint(COUPLING_VARIANCE_MIN, COUPLING_VARIANCE_MAX)
        temperature = units(random.randint(TEMPERATURE_MIN, TEMPERATURE_MAX), units.degC)
        height = random.randint(HEIGHT_MIN, HEIGHT_MAX)

        # Information about building parameters
        print "building %i" % i, ":"
        print "- size:", size
        print "- height:", height
        print "- power:", power
        print "- hysteresis:", hysteresis
        print "- coupling:", coupling
        print "- temperature:", temperature

        building = sim.thermal.add(ThermalProcess.room('building_%s' % i,
                                                       size*units.meter*units.meter,
                                                       height*units.meter,
                                                       units.convert(temperature, units.kelvin)))

        # The controller need this method /!\ hack due to the new version of greedsim related to agreflex
        building.thermal_volumic_capacity = building._thermal_capacity * building._mass

        coupling_outside = sim.thermal.add(ThermalCoupling('building %i to outside' % i,
                                                           coupling*units.thermal_conductivity,
                                                           building,
                                                           outside))

        heater = sim.electrical.add(ElectroThermalHeaterCooler('heater %i' % i,
                                                               power*units.watt,
                                                               1.0,
                                                               building))

        sim.electrical.attach(bus0, heater)

        # Parameter for the Agregator
        sim.agregator.cost_reference = COST
        sim.agregator.day_sigma = DAY_SIGMA
        sim.agregator.period_sigma = SLOT_SIGMA
        sim.agregator.cost_sigma = COST_SIGMA

        # Thermal heater with controller
        forecastController = sim.agregator.add(ForecastController('forecastController %i' % i,
                                                                  temperature,
                                                                  hysteresis,
                                                                  building,
                                                                  heater,
                                                                  'on',  # default: switch on
                                                                  DECISION_DURATION_SLOT,  # decision step
                                                                  DURATION_MINUTE_SLOT,
                                                                  solver=SOLVER))
        forecastController.add(outside, coupling_outside)

        lstForecast += [forecastController]
        lstHeater += [heater]
        lstTemperatureMonitoring += [building]


    sim.agregator.decision_time = DECISION_DURATION_SLOT
    sim.agregator.outside_process = outside

    lstTemperatureMonitoring += [sim.agregator]

    # Create a plot recorder that records the temperatures of all thermal processes.
    temp = PlotRecorder('temperature', units.minute, units.degC)
    sim.record(temp, lstTemperatureMonitoring)

    # Create a plot recorder that records the power used by the electrical heater.
    power = PlotRecorder('delta_energy', units.minute, units.joule)
    sim.record(power, lstHeater)

    # Create a plot recorder that records the cost
    cost = PlotRecorder('_instant_cost', units.minute, units.radian)
    sim.record(cost, [lstForecast[0]])

    error = PlotRecorder('error', units.minute, units.degC)
    sim.record(error, lstForecast)

    average_temperature = PlotRecorder('average_temperature', units.minute, units.degC)
    sim.record(average_temperature, lstForecast)


    # Simulate
    sim.reset()
    sim.run(DURATION_TIME, DURATION_MINUTE_SLOT)

    for forecastController in lstForecast:
        print "+-------------------------------"
        print "| - Nb optimisation:\t", forecastController.countOptimization
        print "| - Sigma error:\t", forecastController.total_error
        print "| - Sigma error abs:\t", forecastController.total_absolute_error
        print "| - Total power:\t", forecastController.total_power()
        print "| - Total cost:\t\t", forecastController.total_cost()
        print "+-------------------------------"

    # Outputs

    path = './forecastcontroller/output/'
    if sub_path is not None:
        path += '{}/'.format(sub_path)

    if not os.path.exists(path):
        os.makedirs(path)

    FigureSaver(temp, "Temperature").save('{}temperature.png'.format(path))
    FigureSaver(power, "Power").save('{}heater.png'.format(path))
    FigureSaver(cost, "Cost").save('{}cost.png'.format(path))
    FigureSaver(error, "Erreur").save('{}error.png'.format(path))
    FigureSaver(average_temperature, "Moyenne").save('{}average_temperature.png'.format(path))
