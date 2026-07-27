# Design Specifications for RASTIC Plastic Recycler v1.0
This system is an open source elaboration on the ArtMe 3D Mk3S+ plastic extruder. The primary changes made with this system include a more streamlined, boutique operating system; improved filament tolerance monitoring and related control feedback systems; a more responsive and multifunction interface; WiFi functionality to record output statistics; and integrated system controls and open source designs for plastic preprocessing for quality enhancement, including a novel preheating system to dehydrate input stock.

## Control System Features
**This section is for development specs**
This system runs on a BigTreeTech SKR2 board with standard motor drivers. All system controls will be managed through the standard BigTreeTech pin-outs, indicated below in the feature I/O descriptions. The listed priority of each feature is the RTOS priority, which is based on the safety requirements of the system such that high priority features will supercede others for safety checks in the RTOS scheduler.

Main features of control system:
- Motor control 
  - Extruder (High priority)
  - Puller (Medium priority)
  - Spooler (Medium priority)
- Preheater control (Highest priority!!!)
  - Preheater
  - Temperature sensor
- Heater control (Highest priority!!!)
  - Heater
  - Temperature sensor
- Tolerance feedback mechanisms (Medium priority)
  - Plastic droop mechanism (aka. arm sensor)
  - Plastic diameter magnetic sensor (aka. magnetic sensor)
- I/O
  - CYD (Low priority)
  - Encoders (Medium priority)
    - Extruder speed
    - Puller speed
    - Heater temp.
    - Preheat temp.
    - General encoder (settings) (Low priority)
  - Buttons
    - Emergency stop (kill all systems during extrusion. If in preheat phase, stops preheat) (Highest priority!!!)
    - Pause (keep heat on, stop extrusion and pulling. When not extruding, also starts preheat phase) (High priority)
    - Begin extrusion (Medium priority)
    - General encoder button (settings) (Low priority)
  - Default operation & presets
- **Optional**: Shredder and sifter command interface (Low priority)