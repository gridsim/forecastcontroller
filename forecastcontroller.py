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

sys.path.append(os.path.abspath("../"))

from gridsim.unit import units
from gridsim.simulation import Simulator
from gridsim.recorder import PlotRecorder
from gridsim.thermal.element import TimeSeriesThermalProcess
from gridsim.thermal.core import ThermalProcess, ThermalCoupling
from gridsim.electrical.network import ElectricalPQBus, \
    ElectricalTransmissionLine
from gridsim.electrical.loadflow import DirectLoadFlowCalculator
from gridsim.timeseries import SortedConstantStepTimeSeriesObject, TimeSeriesObject
from gridsim.iodata.input import CSVReader
from gridsim.iodata.output import FigureSaver

from agregator import ForecastController, ElectroThermalHeaterCooler, AgregatorSimulator


# Create simulator.
sim = Simulator()
sim.electrical.load_flow_calculator = DirectLoadFlowCalculator()

####################################################################################
# Basic configuration :
#
START_TIME = 0
DURATION_TIME = 10 * units.day #1 * units.year
DECISION_DURATION_STEP = 1 * units.day
PERIOD_STEP = 30 * units.minute

# Buildings coupling with outdoor
TEMPERATURE_INIT_REF = 20
COUPLING_OUTSIDE_REF = 20

# Nb buildings with devices and controllers
MAX_INDEX_NB_DEVICES = 1
MAX_DAY_HISTORIC = 20  # historic for corrections

# Example of costs
COST = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 2.0, 2.0, 2.0, 2.0,
        5.0, 5.0, 7.0, 7.0, 10.0, 10.0, 10.0, 10.0, 8.0, 8.0,
        4.0, 4.0, 3.0, 3.0, 4.0, 4.0, 6.0, 6.0, 8.0, 8.0,
        5.0, 5.0, 2.0, 2.0, 1.0, 1.0, 0.0, 0.0]

assert(
    units.value(units.convert(DECISION_DURATION_STEP, units.unit(PERIOD_STEP))) / units.value(PERIOD_STEP) == len(COST)
)

# Outside thermal element
outside = sim.thermal.add(TimeSeriesThermalProcess('Outside Temperature', SortedConstantStepTimeSeriesObject(CSVReader()),
                                                   './data/example_time_series.csv',
                                                   lambda t: t*units.hour,
                                                   temperature_calculator=
                                                   lambda t: units.convert(units(t, units.degC),
                                                                           units.kelvin)))

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
    size = random.randint(500, 4000)
    power = random.randint(500, 4000)
    hysteresis = random.randint(1, 3)
    coupling = COUPLING_OUTSIDE_REF + random.randint(-15, 20)
    temperature = units(TEMPERATURE_INIT_REF + random.randint(-5, 10), units.degC)

    # Information about building parameters
    print "building %i" % i, ":"
    print "- size:", size
    print "- power:", power
    print "- hysteresis:", hysteresis
    print "- coupling:", coupling
    print "- temperature:", temperature

    building = sim.thermal.add(ThermalProcess.room('building_%s' % i,
                                                   size*units.meter*units.meter,
                                                   2.5*units.meter,
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

    # Thermal heater with controller
    forecastController = sim.agregator.add(ForecastController('forecastController %i' % i,
                                                              temperature,
                                                              hysteresis,
                                                              building,
                                                              heater,
                                                              'on',  # default: switch on
                                                              DECISION_DURATION_STEP,  # decision step
                                                              PERIOD_STEP))
    forecastController.add(outside, coupling_outside)
    forecastController.max_day_historic = MAX_DAY_HISTORIC

    lstForecast += [forecastController]
    lstTemperatureMonitoring += [building]
    lstHeater += [heater]


sim.agregator.decision_time = DECISION_DURATION_STEP
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

mean = PlotRecorder('mean', units.minute, units.degC)
sim.record(mean, lstForecast)


# Simulate
sim.reset()
sim.run(DURATION_TIME, PERIOD_STEP)

for forecastController in lstForecast:
    print "+-------------------------------"
    print "| - Nb optimisation:\t", forecastController.countOptimization
    print "| - Sigma error:\t", forecastController.total_error
    print "| - Sigma error abs:\t", forecastController.total_absolute_error
    print "| - Total power:\t", forecastController.total_power()
    print "| - Total cost:\t\t", forecastController.total_cost()
    print "+-------------------------------"

# Outputs
if not os.path.exists('./forecastcontroller/output'):
    os.makedirs('./forecastcontroller/output')
FigureSaver(temp, "Temperature").save('./forecastcontroller/output/temperature.png')
FigureSaver(power, "Power").save('./forecastcontroller/output/heater.png')
FigureSaver(cost, "Cost").save('./forecastcontroller/output/cost.png')
FigureSaver(error, "Erreur").save('./forecastcontroller/output/error.png')
FigureSaver(mean, "Moyenne").save('./forecastcontroller/output/mean.png')
