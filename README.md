RedSensorToCloud
================

This package provides components that can be used to interact with the sensors
and actuators on the mangOH Red DV3 and/or push that data to Sierra Wireless's
AirVantage(tm) cloud service.

There are two .adef files provided for two apps:
- redSensor: Interfaces all sensors with the Legato Data Hub and provides APIs
             for direct function-call-oriented access by client apps.
- redCloud: Takes data from the Data Hub and pushes it to AirVantage.
