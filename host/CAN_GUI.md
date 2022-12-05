# CAN GUI
Host GUI to tune gains and inspect position, velocity, torque, currents, and voltages in real-time via CAN.

## RUN
```
    python can_gui.py
```
## TODO
- Tune TYI Motor (5010 KV340)
- Finish plotting remaining current & voltage plots
- Fix velocity plot
- Better code organization

## MOTOR GAINS
- MAD Components (4006 EEE KV250)
    - Current: Kp=1.5, Ki=0.05
    - Position: Kp=0.4, Kd=0.03

- TYI Motor (5010 KV340)
    - Current: Kp, Ki
    - Position: Kp, Ki

