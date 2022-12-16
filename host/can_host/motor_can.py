import can_protocol
import time
from math import sin, cos
class ESCControl: 
    def __init__(self, device_id):
        ser = can_protocol.SerialTransport("COM9", baudrate=1000000)
        self.motor = can_protocol.RecoilMotorController(transport=ser, device_id=device_id)
        ser.start()

        self.motor.setMode(can_protocol.RecoilMotorController.MODE_IDLE)
        time.sleep(0.5)

        self.target_fcn = "Idle"

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
    
    def updateGeneral(self):
        self.motor.getGeneral()
        # replace with general 8-byte cmd later (pos, vel, torque, kp, kd)
        self.motor.getTargetPosition()
        self.motor.getTargetVelocity()
        self.motor.getTargetTorque()

    def setTargetFcn(self, mode="Idle"):
        self.target_fcn = mode

    def setTarget(self):    
        if (self.motor.mode == can_protocol.RecoilMotorController.MODE_IDLE):
            self.motor.getGeneral()
            self.motor.setTargetPosition(self.motor.params['position_measured'])
            self.motor.setTargetVelocity(0)
            self.motor.setTargetTorque(0)
            self.motor.setTargetIQ(0)
            self.target_fcn = "Idle"

        elif (self.motor.mode == can_protocol.RecoilMotorController.MODE_POSITION):
            target_position = 0
            target_velocity = 0
            match self.target_fcn:
                case "Idle":
                    return
                case "Step":
                    target_position = 3.0
                    target_velocity = 0.0 
                case "Neg_Step":
                    target_position = -3.0
                    target_velocity = 0
                case "sin2w": 
                    target_position = 5*sin(2*time.time())
                    target_velocity = 10*cos(2*time.time())
                case "sin4w":
                    target_position = 5*sin(4*time.time())
                    target_velocity = 20*cos(4*time.time())
                case "sin8w":
                    target_position = 5*sin(8*time.time())
                    target_velocity = 40*cos(8*time.time())
                case "rect":
                    # period = 2 # [s]
                    # if time.time_ns() % (period*10**9) == 0: nextTarget = -self.previousTarget()
                    pass
                case _:
                    print(f"Invalid target position function: {self.target_fcn}")
                    return
                    
            self.motor.setTargetPosition(target_position)
            self.motor.setTargetVelocity(target_velocity)
        
        elif (self.motor.mode == can_protocol.RecoilMotorController.MODE_TORQUE):
            torque_target = 0
            match self.target_fcn:
                case "Idle":
                    return
                case "Step":
                    torque_target = 1
                case "sin2w":
                    torque_target = 0.2*sin(2*time.time())
                case _:
                    print(f"Invalid target position function: {self.target_fcn}")
            # self.motor.accessCAN({'can_id': self.motor.CAN_ID_TORQUE_TARGET, 'torque': torque_target})
            self.motor.setTargetTorque(torque_target)

        elif (self.motor.mode == can_protocol.RecoilMotorController.MODE_OPEN_IDQ):
            target_iq = 0
            match self.target_fcn:
                case "Idle":
                    return
                case "Step":
                    target_iq = 1
                case "Neg_Step":
                    target_iq = -1
                case "sin2w":
                    target_iq = 1*sin(2*time.time())
                case "sin4w":
                    target_iq = 1*sin(4*time.time())
                case "sin8w":
                    target_iq = 1*sin(8*time.time())
                case "rect":
                    pass
                case _:
                    print(f"Invalid target position function: {self.target_fcn}")
                    return

            self.motor.setTargetID(0)
            self.motor.setTargetIQ(target_iq)
 
    def saveToFlash(self):
        self.motor.saveToFlash()

    def accessParam(self, params):
        '''
            Generic setter / getter
            TODO: 
            - update to use self.motor.accessCAN
            - relate params.pop('name') to CAN_ID
            - automatically determine what params to print for getter - use can_protocol callback?
        '''
        fcnName = params.pop('name')
        operation = params.pop('operation', 'get')
        getFcnName = "self.motor.get" + fcnName + "()"
        setFcnName = "self.motor.set" + fcnName

        eval(getFcnName)

        for k,v in params.items():
            params[k] = float(v) if (v is not None and isinstance(float(v), (float, int))) else self.motor.params[k]

        print(fcnName, params)
        
        if operation == 'set': 
            eval(setFcnName)(*params.values())