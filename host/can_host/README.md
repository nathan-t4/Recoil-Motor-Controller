# CAN GUI
Host GUI to tune gains and inspect position, velocity, torque, currents, and voltages in real-time via CAN.

## RUN
```
    python can_gui.py
```
## GUI TODO
- [x] Tune TYI Motor (5010 KV340)
- [ ] Finish plotting remaining current & voltage plots
- [x] Fix velocity plot
- [ ] Estimate kt for gripper motor (4006 EEE KV250) -> kt = tau / iq
- [ ] Fix save to flash
- [ ] Better code organization

## FIRMWARE TODO
- [x] Fix encoder velocity estimate 
- [x] Fix position PD loop - poor position/velocity tracking (velocity filter coeff, velocity limit, torque limit...?)
- [x] Test torque control
- [ ] Better encoder filter?
- [ ] Implement general CAN commands (pos, vel, torque, kp, kd) (tested rx, need to check tx)
- [ ] Implement velocity control
- [ ] Fix / tune pos, vel, torque limits to prevent wrap-around
- [ ] Remap CAN func IDs
- [ ] Improve calibration process?

## MOTOR GAINS
- MAD Components (4006 EEE KV250)
    - Current: Kp=1.5, Ki=0.05
    - Position: Kp=0.4, Kd=0.03

- TYI Motor (5010 KV340)
    - Current: Kp=2.5, Ki=0.0005
    - Position: Kp=4.0, Kd=0.05

