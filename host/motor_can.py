import can_protocol
import time
from math import sin

'''
[TODO]: 
    implement UI to set current Kp Ki in real-time
    implement UI to set position Kp Ki Kd in real-time
'''
class ESCControl: 
    def __init__(self):
        ser = can_protocol.SerialTransport("COM9", baudrate=1000000)
        self.motor = can_protocol.RecoilMotorController(transport=ser, device_id=1)
        ser.start()

        self.motor.setMode(can_protocol.RecoilMotorController.MODE_IDLE)
        time.sleep(0.5)

        self.phase_current_measured = [0, 0, 0]
        self.bus_voltage_measured = 0
        self.target_position_fcn = "Idle"

    def commandMode(self, mode):
        self.motor.setMode(mode)
        time.sleep(0.5)

    def update(self):
        self.updateCurrent()
        self.updateVoltage()
        self.updatePosition()
    
    def updateCurrent(self):
        self.motor.getIQ()
        self.motor.getID()
        # self.motor.getPhaseCurrent()
        # self.motor.getIab()
        
    def updateVoltage(self):
        self.motor.getVQ()
        self.motor.getVD()
        # self.motor.getPhaseVoltage()
        # self.motor.getVab()      
    
    def updatePosition(self):
        self.motor.getPosition()
        self.motor.getTargetPosition()
    
    def setTargetPositionFcn(self, mode="Idle"):
        self.target_position_fcn = mode

    def setTargetPosition(self):
        if (self.motor.mode == can_protocol.RecoilMotorController.MODE_IDLE):
            self.motor.getPosition()
            self.motor.setTargetPosition(self.motor.params.position_measured)
            self.target_position_fcn = "Idle"

        elif (self.motor.mode == can_protocol.RecoilMotorController.MODE_POSITION):
            match self.target_position_fcn:
                case "Idle":
                    return
                case "Step":
                    self.motor.setTargetPosition(1)
                case "sin2w":
                    self.motor.setTargetPosition(5*sin(2*time.time()))
                case "sin4w":
                    self.motor.setTargetPosition(5*sin(4*time.time()))
                case "sin8w":
                    self.motor.setTargetPosition(5*sin(8*time.time()))
                case "rect":
                    pass
                case _:
                    print(f"Invalid target position function: {self.target_position_fcn}")
        
    def updateVelocity(self):
        self.motor.getVelocity()
        
    def updateTorque(self):
        self.motor.getTorque()
        self.motor.getTargetTorque()