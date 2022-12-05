import can_protocol
import time
from math import sin
class ESCControl: 
    def __init__(self, device_id):
        ser = can_protocol.SerialTransport("COM9", baudrate=1000000)
        self.motor = can_protocol.RecoilMotorController(transport=ser, device_id=device_id)
        ser.start()

        self.motor.setMode(can_protocol.RecoilMotorController.MODE_IDLE)
        time.sleep(0.5)

        self.phase_current_measured = [0, 0, 0]
        self.bus_voltage_measured = 0
        self.target_fcn = "Idle"
        self.previousTarget = 0

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
    
    def updateVelocity(self):
        self.motor.getVelocity()
        
    def updateTorque(self):
        self.motor.getTorque()
        self.motor.getTargetTorque()

    def setTargetFcn(self, mode="Idle"):
        self.target_fcn = mode

    def setTarget(self):
        if (self.motor.mode == can_protocol.RecoilMotorController.MODE_IDLE):
            self.motor.getPosition()
            self.motor.setTargetPosition(self.motor.params.position_measured)
            self.motor.setTargetIQ(0)
            self.target_fcn = "Idle"

        elif (self.motor.mode == can_protocol.RecoilMotorController.MODE_POSITION):
            match self.target_fcn:
                case "Idle":
                    return
                case "Step":
                    self.motor.setTargetPosition(1.0)
                case "sin2w": 
                    self.motor.setTargetPosition(5*sin(2*time.time()))
                case "sin4w":
                    self.motor.setTargetPosition(5*sin(4*time.time()))
                case "sin8w":
                    self.motor.setTargetPosition(5*sin(8*time.time()))
                case "rect":
                    # period = 2 # [s]
                    # if time.time_ns() % (period*10**9) == 0: nextTarget = -self.previousTarget()
                    pass
                case _:
                    print(f"Invalid target position function: {self.target_fcn}")

        elif (self.motor.mode == can_protocol.RecoilMotorController.MODE_OPEN_IDQ):
            self.motor.setTargetID(0)
            match self.target_fcn:
                case "Idle":
                    return
                case "Step":
                    self.motor.setTargetIQ(1)
                case "sin2w":
                    self.motor.setTargetIQ(1*sin(2*time.time()))
                case "sin4w":
                    self.motor.setTargetIQ(1*sin(4*time.time()))
                case "sin8w":
                    self.motor.setTargetIQ(1*sin(8*time.time()))
                case "rect":
                    period = 0.5 # [s]
                    y = -1 if time.time_ns() % (period*10**9) == 0 else 1
                    self.motor.setTargetIQ(y)
                case _:
                    print(f"Invalid target position function: {self.target_fcn}")
        
    def setCurrentParams(self, kp='', ki=''):
        self.motor.getCurrentParams()
        kp = self.motor.params.current_kp if kp == '' or not isinstance(float(kp), (float, int)) else float(kp)
        ki = self.motor.params.current_ki if ki == '' or not isinstance(float(ki), (float, int)) else float(ki)
        print(f"New Current Params: Kp: {kp}, Ki: {ki}")
        self.motor.setCurrentParams(kp, ki)
    
    def getCurrentParams(self):
        self.motor.getCurrentParams()
        print(f"Current Params - Kp: {self.motor.params.current_kp}, Ki: {self.motor.params.current_ki}")
        
    def setPositionParams(self, kp='', kd=''):
        self.motor.getPositionParams()
        kp = self.motor.params.position_kp if kp == '' or not isinstance(float(kp), (float, int)) else float(kp)
        kd = self.motor.params.position_kd if kd == '' or not isinstance(float(kd), (float, int)) else float(kd)
        print(f"New Position Params - Kp: {kp}, Kd: {kd}")
        self.motor.setPositionParams(kp, kd)

    def getPositionParams(self):
        self.motor.getPositionParams()
        print(f"Position Params - Kp: {self.motor.params.position_kp}, Kd: {self.motor.params.position_kd}")

    def setTorqueLimits(self, lower='', upper=''):
        self.motor.getTorqueLimits()
        lower = self.motor.params.torque_limit_lower if lower == '' or not isinstance(float(lower), (float, int)) else float(lower)
        upper = self.motor.params.torque_limit_upper if upper == '' or not isinstance(float(upper), (float, int)) else float(upper)
        print(f"New Torque Limits - Lower: {lower}, Upper: {upper}")
        self.motor.setTorqueLimits(lower, upper)

    def getTorqueLimits(self):
        self.motor.getTorqueLimits()
        print(f"Torque Limits - Lower: {self.motor.params.torque_limit_lower}, Upper: {self.motor.params.torque_limit_upper}")

    def saveToFlash(self):
        self.motor.saveToFlash()