# Cart project

This repo contains fixes and additional features for the can driver v2. This branch is mainly used for testing the `Sensor Controller Board V1.0` and the CAN communication. I will add some feedback for the next iteration of the PCB. 


## TODOS:
- Test the custom PCB and the MCP2515 module for CAN communication between two MCUs. 
- Add wake up from sleep mode feature, for minimal power consumtion.
- Fix the filter and listen on certain ID logic, cleaning up current.

### Findings and feedback for the `Sensor Controller Board V1.0`

- The U1 component (MCP2515) seems to work fine with 3.3 voltage VDD input. However comparing TJA1050 (external module) with the TJA1051T (component on PCB board). TJA1050 work with VCC of both 3.3V and 5V, while TJA1051T works for 4.5V ~ 5.5V. 
