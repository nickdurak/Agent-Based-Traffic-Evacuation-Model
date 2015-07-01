Traffic Light Timing for the Evacuation of an Urban Area
========================================================

This is a simulation of cars driving in an urban area when an evacuation is
signalled and all drivers must leave the area. The evacuation is due to a
chlorine spill.

The simulation is designed to be run in the HEEDS optimization software.

The simulation runs in the [NetLogo](https://github.com/NetLogo/NetLogo) environment

Files
-----

* history.txt - Outlines changes made by version number
* NLPP.py - NetLogo Pre-processor. This is used to strip out debug printing and other GUI only code for running headless.
* traffic_evac_conc - Text file defining the chlorine plume at 64 points on the map at 5 minute intervals. Created from ALOHA data
* traffic_evac_experiment_4.xml - Settings to run the simulation in NetLogo BehaviorSpace
* traffic_evac_garages - Defines the locations of the parking garages on the map
* traffic_evac_lights - Defines the traffic light timing and synchronization to use in the evacuation mode. This should be modified by the optimization software (HEEDS)
* traffic_evac_signs - Defines the locations of emergency electronic signs that can instruct drivers to turn on their radios for evacuation instructions
* traffic_evacation.nlogo - NetLogo simulation.
* traffic_pre - Light timing and synchronization used before evacuation. This sets up "normal" driving conditions

Changelog
---------

[history.txt]

