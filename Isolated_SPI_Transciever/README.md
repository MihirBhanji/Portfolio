Developed an Isolated SPI transciever to communicate between the Master BMS and Distributed BMS

Design
- Built around BQ79600 IC
- 4 layers: Sig,GND,GND,Sig
- Provides Galvanic isolation
- Ring architecture and reverse wakeup

Physical Spec:
- 29.21 mm x 69.215 mm 
- Molex Connectors
- Pin headers to function as a daughter board

Firmware
- Wakes up the Bridge & stack(BQ79616)
- Auto assigns addresses, starts their ADCs
- Prints cell voltages/temps on Serial monitor

