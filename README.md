As a hobby project, a driving controller for an electric-powered children's car is developed.
##Its main objectives are:
- Smooth PWM control of the electric motors power at starting and stopping to ensure smooth running and minimize abrupt loads of mechanical gearbox units.
- Accelerator pedal support, both with a conventional switch and with variable electrical output potential (resistive or optoelectronic).
- Supports forward and reverse mode switching with external handle control.
- Time delay when switching the polarity of the motors (waiting for the rotors to stop) to avoid overloading electrical circuits and blocking mechanical components.
- Battery charge level monitoring.
- Prohibition of driving when the battery is empty to prevent deep discharge.
- Indication of the charge level and the state of connection of the charger on the external module of the seven-segment LED display.
- Audible signaling of events - switching of driving direction, critical battery charge, charger connection.


The vehicle is equipped with two RS550 motors with 12 volt operating voltage. Practical measurements showed current consumption at full load up to 18A. 

![pcb1](/picture-02.jpg)

As a power switching part of the PWM controller were used popular low-voltage N-channel field-effect transistors IRF3205 with low internal junction resistance. 
Two transistors are used in parallel to reduce heat dissipation and increase reliability. The power transistors are shunted by a Schottky diode 25CTQ045 to prevent breakdown by motor reverse current throws. 
To match the voltage level of IRF3205 gates and their fast saturation for operation in switching mode, an integrated driver IR4427S connected to Atmega microcontroller is used. 

##PCB
![pcb1](/controller-PCB.gif)

![pcb2](/picture-01.jpg)
