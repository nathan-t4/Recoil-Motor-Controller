import struct
import time
import math
import threading

import serial
from bitstring import Bits, BitArray
import motor
# from dotxboxcontroller import XboxController, Hand

# device_id 4 bits
# func_id   7 bits

class Handler:
    def __init__(self, frame, device, callback=None):
        self.frame = frame
        self.device = device
        self.callback = callback
    
debug = 0

class CANFrame:
    CAN_ID_STANDARD = 0
    CAN_ID_EXTENDED = 1
    
    CAN_FRAME_REMOTE = 1
    CAN_FRAME_DATA = 1
    
    def __init__(self, device_id=0, func_id=0, size=0, data=b"",
                 id_type=CAN_ID_STANDARD, frame_type=CAN_FRAME_REMOTE):
        self.device_id = device_id
        self.func_id = func_id
        self.id_type = id_type
        self.frame_type = frame_type
        self.size = size
        self.data = data

class SerialTransport:
    def __init__(self, port="COM9", baudrate=1000000):
        self.port = port
        self._ser = serial.Serial(port=self.port, baudrate=baudrate, timeout=None)
        self.handlers = []

    def transmitCANFrame(self, frame):
        can_id = (frame.func_id << 4) | frame.device_id
        # Big endian here to be compatible with CAN ID arbitration order
        header = struct.pack(">HBB", can_id, frame.size, frame.frame_type)
        fill = b'\x00' * (8 - len(frame.data))
        buffer = header + frame.data + fill
        if debug:
            print("[DEBUG] transmit", buffer)
        self._ser.write(buffer)

    def transmitReceiveCANFrame(self, handler):
        self.handlers.append(handler)
        self.transmitCANFrame(handler.frame)

    def receiveCANFrame(self):
        while True:
            frame = CANFrame()
            if debug:
                print("[DEBUG] receiving frame...")
            buffer = self._ser.read(4)
            if not buffer:
                continue
            if debug:
                print("[DEBUG] receive", buffer)
            # Big endian here to be compatible with CAN ID arbitration order
            can_id, frame.size, frame.frame_type = struct.unpack(">HBB", buffer)
            frame.device_id = can_id & 0x0F
            frame.func_id = can_id >> 4
            buffer = self._ser.read(frame.size)
            frame.data = buffer

            for handler in self.handlers:
                if handler.frame.device_id == frame.device_id and handler.frame.func_id == frame.func_id:
                    handler.device.handleRX(frame)
                    if handler.callback:
                        handler.callback(handler.device, frame)
                    self.handlers.remove(handler)


    def start(self):        
        t = threading.Thread(target=self.receiveCANFrame)
        t.start()


class RecoilMotorController:
    '''
        TODO
        - generic callback to handle rx
        - remap all can_id (ex. CAN_ID_POSITION -> position_measured (32 bit), position_target (32 bit))
        - fix accessParam() in motor_can.py and delete all get/set fcns
    '''

    CAN_ID_ESTOP              = 0x00
    CAN_ID_ID                 = 0x01
    CAN_ID_VERSION            = 0x02
    CAN_ID_HEARTBEAT          = 0x04
    CAN_ID_GENERAL            = 0x05

    CAN_ID_MODE               = 0x10
    CAN_ID_FLASH              = 0x11

    CAN_ID_TORQUE_MEASURED    = 0x20
    CAN_ID_TORQUE_TARGET      = 0x21
    CAN_ID_VELOCITY_MEASURED  = 0x22
    CAN_ID_VELOCITY_TARGET    = 0x23
    CAN_ID_POSITION_MEASURED  = 0x24
    CAN_ID_POSITION_TARGET    = 0x25
    CAN_ID_POSITION_KP_KD     = 0x26
    CAN_ID_POSITION_KI        = 0x27
    CAN_ID_IQ_KP_KI           = 0x28
    CAN_ID_ID_KP_KI           = 0x29

    CAN_ID_BUS_VOLTAGE        = 0x30
    CAN_ID_POSITION_LIMIT     = 0x31
    CAN_ID_VELOCITY_LIMIT     = 0x32
    CAN_ID_TORQUE_LIMIT       = 0x33

    CAN_ID_MOTOR_SPEC         = 0x40
    CAN_ID_MOTOR_FLUX_OFFSET  = 0x41
    CAN_ID_ENCODER_N_ROTATION = 0x42

    CAN_ID_CURRENT_DQ         = 0x44
    CAN_ID_CURRENT_AB         = 0x45

    CAN_ID_CURRENTCONTROLLER_IQ = 0x50
    CAN_ID_CURRENTCONTROLLER_ID = 0x51
    CAN_ID_CURRENTCONTROLLER_VQ = 0x52
    CAN_ID_CURRENTCONTROLLER_VD = 0x53

    CAN_ID_PING               = 0x7F
    
    MODE_DISABLED           = 0x00
    MODE_IDLE               = 0x01
    MODE_CALIBRATION        = 0x05
    MODE_TORQUE             = 0x10
    MODE_VELOCITY           = 0x11
    MODE_POSITION           = 0x12
    MODE_OPEN_VDQ           = 0x22
    MODE_OPEN_VALPHABETA    = 0x23
    MODE_OPEN_VABC          = 0x24
    MODE_OPEN_IDQ           = 0x25
    MODE_DEBUG              = 0x80

    def __init__(self, transport, device_id):
        self.transport = transport
        self.device_id = device_id 
        self.params = motor.MotorParams().params 
        self.can2params = motor.MotorParams().can2params
        self.mode = self.MODE_DISABLED

        # init parameters
        self.getPositionLimits()
        self.getVelocityLimits()
        self.getTorqueLimits()

    def getGeneral(self, callback=None):
        frame = CANFrame(self.device_id, self.CAN_ID_GENERAL, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self, callback))

    def setGeneral(self, position, velocity, kp, kd, torque):
        # TODO: test
        def float2uint(x, x_min, x_max, bits):
            span = x_max - x_min
            offset = x_min
            return int((x-offset) * float((1<<bits)-1)) / span
        
        position = float2uint(position, self.params['position_limit_lower'], self.params['position_limit_upper'], 16)
        velocity = float2uint(velocity, self.params['velocity_limit_lower'], self.params['velocity_limit_upper'], 12)
        kp = float2uint(kp, self.params['kp_limit_lower'], self.params['kp_limit_upper'], 12)
        kd = float2uint(kd, self.params['kd_limit_lower'], self.params['kd_limit_upper'], 12) 
        torque = float2uint(torque, self.params['torque_limit_lower'], self.params['torque_limit_upper'], 12)

        # little-endian
        msg = BitArray(uintle=position, length=16) + BitArray(uintle=velocity, length=12) + BitArray(uintle=kp, length=12) + \
              BitArray(uintle=kd, length=12) + BitArray(uintle=torque, length=12)
        
        frame = CANFrame(self.device_id, self.CAN_ID_GENERAL, 8, msg)
        self.transport.transmitCANFrame(frame)

    def saveToFlash(self, callback=None):
        frame = CANFrame(self.device_id, self.CAN_ID_FLASH, 1, b'1')
        self.transport.transmitReceiveCANFrame(Handler(frame, self, callback))
        print("Saved to flash")
    
    def getMode(self, callback=None):
        frame = CANFrame(self.device_id, self.CAN_ID_MODE, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self, callback))
    
    def setMode(self, mode):
        frame = CANFrame(self.device_id, self.CAN_ID_MODE, 1, struct.pack("<B", mode))
        self.transport.transmitCANFrame(frame)

    def getPosition(self, callback=None):
        frame = CANFrame(self.device_id, self.CAN_ID_POSITION_MEASURED, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self, callback))
    
    def getTargetPosition(self, callback=None):
        frame = CANFrame(self.device_id, self.CAN_ID_POSITION_TARGET, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self, callback))
    
    def setTargetPosition(self, position):
        frame = CANFrame(self.device_id, self.CAN_ID_POSITION_TARGET, 4, struct.pack("<f", position))
        self.transport.transmitCANFrame(frame)
    
    def getVelocity(self, callback=None):
        frame = CANFrame(self.device_id, self.CAN_ID_VELOCITY_MEASURED, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self, callback))

    def getTargetVelocity(self, callback=None):
        frame = CANFrame(self.device_id, self.CAN_ID_VELOCITY_TARGET, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self, callback))

    def setTargetVelocity(self, velocity):
        frame = CANFrame(self.device_id, self.CAN_ID_VELOCITY_TARGET, 4, struct.pack("<f", velocity))
        self.transport.transmitCANFrame(frame)

    def getTargetTorque(self, callback=None):
        frame = CANFrame(self.device_id, self.CAN_ID_TORQUE_TARGET, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self, callback))

    def setTargetTorque(self, torque):
        frame = CANFrame(self.device_id, self.CAN_ID_TORQUE_TARGET, 4, struct.pack("<f", torque))
        self.transport.transmitCANFrame(frame)
        
    def getIQ(self):
        frame = CANFrame(self.device_id, self.CAN_ID_CURRENTCONTROLLER_IQ, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self))

    def getID(self):
        frame = CANFrame(self.device_id, self.CAN_ID_CURRENTCONTROLLER_ID, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self))

    def setTargetIQ(self, iq):
        frame = CANFrame(self.device_id, self.CAN_ID_CURRENTCONTROLLER_IQ, 4, struct.pack("<f", iq))
        self.transport.transmitCANFrame(frame)
        
    def setTargetID(self, id):
        frame = CANFrame(self.device_id, self.CAN_ID_CURRENTCONTROLLER_ID, 4, struct.pack("<f", id))
        self.transport.transmitCANFrame(frame)
    
    def getPhaseCurrents(self):
        pass

    def getIab(self):
        pass
    
    def getVQ(self):
        frame = CANFrame(self.device_id, self.CAN_ID_CURRENTCONTROLLER_VQ, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self))

    def getVD(self):
        frame = CANFrame(self.device_id, self.CAN_ID_CURRENTCONTROLLER_VD, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self))

    def getVbus(self):
        frame = CANFrame(self.device_id, self.CAN_ID_BUS_VOLTAGE, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self))

    def getCurrentParams(self):
        frame = CANFrame(self.device_id, self.CAN_ID_IQ_KP_KI, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self))
    
    def setCurrentParams(self, current_kp, current_ki):
        frame = CANFrame(self.device_id, self.CAN_ID_IQ_KP_KI, 8, struct.pack("<ff", current_kp, current_ki))
        self.transport.transmitCANFrame(frame)
    
    def getPositionParams(self):
        frame = CANFrame(self.device_id, self.CAN_ID_POSITION_KP_KD, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self))
        frame = CANFrame(self.device_id, self.CAN_ID_POSITION_KI, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self))

    def setPositionParams(self, position_kp, position_kd):
        frame = CANFrame(self.device_id, self.CAN_ID_POSITION_KP_KD, 8, struct.pack("<ff", position_kp, position_kd))
        self.transport.transmitCANFrame(frame)
    
    def getTorque(self):
        frame = CANFrame(self.device_id, self.CAN_ID_TORQUE_MEASURED, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self))
    
    def getTargetTorque(self):
        frame = CANFrame(self.device_id, self.CAN_ID_TORQUE_TARGET, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self))
    
    def getPositionLimits(self):
        frame = CANFrame(self.device_id, self.CAN_ID_POSITION_LIMIT, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self))

    def setPositionLimits(self, position_limit_lower, position_limit_upper):
        frame = CANFrame(self.device_id, self.CAN_ID_POSITION_LIMIT, 8, struct.pack("<ff", position_limit_lower, position_limit_upper))
        self.transport.transmitCANFrame(frame)
    
    def getVelocityLimits(self):
        frame = CANFrame(self.device_id, self.CAN_ID_VELOCITY_LIMIT, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self))

    def setVelocityLimits(self, velocity_limit_lower, velocity_limit_upper):
        frame = CANFrame(self.device_id, self.CAN_ID_VELOCITY_LIMIT, 8, struct.pack("<ff", velocity_limit_lower, velocity_limit_upper))
        self.transport.transmitCANFrame(frame)
    
    def getTorqueLimits(self):
        frame = CANFrame(self.device_id, self.CAN_ID_TORQUE_LIMIT, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self))
    
    def setTorqueLimits(self, torque_limit_lower, torque_limit_upper):
        frame = CANFrame(self.device_id, self.CAN_ID_TORQUE_LIMIT, 8, struct.pack("<ff", torque_limit_lower, torque_limit_upper))
        self.transport.transmitCANFrame(frame)

    def getMotorSpecs(self):
        frame = CANFrame(self.device_id, self.CAN_ID_MOTOR_SPEC, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self))

    def feed(self):
        frame = CANFrame(self.device_id, self.CAN_ID_HEARTBEAT, 1, b"0")
        self.transport.transmitCANFrame(frame)

    def ping(self, callback=None):
        frame = CANFrame(self.device_id, self.CAN_ID_PING, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self, callback))

    def accessCAN(self, params={}):
        '''
            Generic getter / setter via CAN
            Example: Set motor torque target to 3.0 [Nm]
                accessCAN({'can_id': CAN_ID_TORQUE_LIMIT, 'torque': 3.0})
            TODO: take remaining params as list?
        '''
        can_id = params.pop('can_id', self.CAN_ID_PING)
        # print(can_id)
        operation = 'get' if params == {} else 'set'
        
        if operation == 'set':
            params_type = '<'
            type_dict = {'float': 'f', 'int': 'B'} # B is unsigned char
            for item in params.values():
                try:
                    params_type += type_dict[type(item).__name__]
                except KeyError:
                    print('Invalid params list')
                    return

            data = struct.pack(params_type, *params.values())

            frame = CANFrame(self.device_id, can_id, len(params), data)
            self.transport.transmitCANFrame(frame)
        else:
            frame = CANFrame(self.device_id, can_id, 0)
            self.transport.transmitReceiveCANFrame(Handler(frame, self))

    def handleRX(self, frame):
        if frame.func_id == self.CAN_ID_GENERAL:
            # TODO: deal with wraparound, fix velocity limits to V_bus*motor_kv?, change can msg format?
            def uint2float(x_int, x_min, x_max, bits):
                span = x_max - x_min
                offset = x_min
                return (float(x_int)*span / float(1<<bits)-1) + offset
            
            rx_data = BitArray(bytes=frame.data)
            p = rx_data[:16]
            v = rx_data[16:24] + BitArray(hex='0x0') + rx_data[24:28]
            t = rx_data[36:40] + rx_data[28:32] + BitArray(hex='0x0') +  rx_data[32:36] 
            # print(rx_data,p,vt)
            position = uint2float(p.unpack('uintle16')[0], self.params['position_limit_lower'], self.params['position_limit_upper'], 16)
            velocity = uint2float(v.unpack('uintle16')[0], self.params['velocity_limit_lower'], self.params['velocity_limit_upper'], 12)
            torque = uint2float(t.unpack('uintle16')[0], self.params['torque_limit_lower'], self.params['torque_limit_upper'], 12)
            self.params["position_measured"] = position
            self.params["velocity_measured"] = velocity
            self.params["torque_measured"] = torque

        if frame.func_id == self.CAN_ID_MODE:
            self.mode = frame.data[0]
            # self.error = frame.data[1]
        
        if frame.func_id == self.CAN_ID_POSITION_MEASURED:
            position_measured, = struct.unpack("<f", frame.data[0:4])
            self.params["position_measured"] = position_measured

        if frame.func_id == self.CAN_ID_POSITION_TARGET:
            position_target, = struct.unpack("<f", frame.data[0:4])
            self.params["position_target"] = position_target

        if frame.func_id == self.CAN_ID_VELOCITY_MEASURED:
            velocity_measured, = struct.unpack("<f", frame.data[0:4])
            self.params["velocity_measured"] = velocity_measured   
            
        if frame.func_id == self.CAN_ID_VELOCITY_TARGET:
            velocity_target, = struct.unpack("<f", frame.data[0:4])
            self.params["velocity_target"] = velocity_target

        if frame.func_id == self.CAN_ID_TORQUE_MEASURED:
            torque_measured, = struct.unpack("<f", frame.data[0:4])
            self.params["torque_measured"] = torque_measured
        
        if frame.func_id == self.CAN_ID_TORQUE_TARGET:
            torque_target, = struct.unpack("<f", frame.data[0:4])
            self.params["torque_target"] = torque_target

        if frame.func_id == self.CAN_ID_POSITION_LIMIT:
            position_limit_lower, position_limit_upper = struct.unpack("<ff", frame.data)
            self.params["position_limit_lower"] = position_limit_lower
            self.params["position_limit_upper"] = position_limit_upper

        if frame.func_id == self.CAN_ID_VELOCITY_LIMIT:
            velocity_limit_lower, velocity_limit_upper = struct.unpack("<ff", frame.data)
            self.params["velocity_limit_lower"] = velocity_limit_lower
            self.params["velocity_limit_upper"] = velocity_limit_upper

        if frame.func_id == self.CAN_ID_TORQUE_LIMIT:
            torque_limit_lower, torque_limit_upper = struct.unpack("<ff", frame.data)
            self.params["torque_limit_lower"] = torque_limit_lower
            self.params["torque_limit_upper"] = torque_limit_upper

        if frame.func_id == self.CAN_ID_CURRENTCONTROLLER_IQ:
            iq_target, iq_measured = struct.unpack("<ff", frame.data)
            self.params["iq_target"] = iq_target
            self.params["iq_measured"] = iq_measured

        if frame.func_id == self.CAN_ID_CURRENTCONTROLLER_ID:
            id_target, id_measured = struct.unpack("<ff", frame.data)
            self.params["id_target"] = id_target
            self.params["id_measured"] = id_measured

        if frame.func_id == self.CAN_ID_CURRENTCONTROLLER_VQ:
            vq_target, = struct.unpack("<f", frame.data[0:4])
            self.params["vq_target"] = vq_target

        if frame.func_id == self.CAN_ID_CURRENTCONTROLLER_VD:
            vd_target, = struct.unpack("<f", frame.data[0:4])
            self.params["vd_target"] = vd_target

        if frame.func_id == self.CAN_ID_BUS_VOLTAGE:
            v_bus, = struct.unpack("<f", frame.data[0:4])
            self.params["v_bus"] = v_bus

        if frame.func_id == self.CAN_ID_IQ_KP_KI:
            kp, ki = struct.unpack("<ff", frame.data)
            self.params["current_kp"] = kp
            self.params["current_ki"] = ki

        if frame.func_id == self.CAN_ID_POSITION_KP_KD:
            kp, kd = struct.unpack("<ff", frame.data)
            self.params["position_kp"] = kp
            self.params["position_kd"] = kd

        if frame.func_id == self.CAN_ID_POSITION_KI:
            ki, = struct.unpack("<f", frame.data[0:4])
            self.params["position_ki"] = ki

        if frame.func_id == self.CAN_ID_MOTOR_SPEC:
            ppairs, kv = struct.unpack("<II", frame.data)
            self.params["pole_pairs"] = ppairs
            self.params["kv"] = kv

        if frame.func_id == self.CAN_ID_PING:
            # print(self.device_id, frame.data[0])
            pass
            
# stick = XboxController(0)

def ping_interrupt_handler(device, frame):
    print(device.device_id, frame.device_id)

def print_mode_handler(device, frame):
    print(device.device_id, frame.data[0], frame.data[1])


def main():
    ser = SerialTransport("COM9", baudrate=1000000)

    motor_0 = RecoilMotorController(transport=ser, device_id=1)
    motor_1 = RecoilMotorController(transport=ser, device_id=2)
    print("ser start")
    ser.start()
    print("setMode to idle")
    motor_0.setMode(RecoilMotorController.MODE_IDLE)
    motor_1.setMode(RecoilMotorController.MODE_IDLE)

    time.sleep(0.5)

    # motor_0.setMode(RecoilMotorController.MODE_POSITION)
    # motor_1.setMode(RecoilMotorController.MODE_POSITION)
    
    # motor_0.ping()
    # motor_1.ping()

    while True:
        # stick.update()
        
        t = time.time()

        
        motor_0.getMode()
        # motor_1.getMode()
        
        
        # val0 = 0
        # val1 = 0
        # val0 = stick.getX(Hand.LEFT) * 5
        # val1 = stick.getY(Hand.RIGHT) * 10
        

        
        motor_0.getPosition()
        motor_1.getPosition()
        motor_0.getTargetPosition()
        motor_1.getTargetPosition()
        # motor_0.setTarget(4*math.sin(2*time.time()))
        # motor_1.setTarget(2*math.sin(time.time()))
        # motor_0.setTarget(0)
        # motor_1.setTarget(0)
        # print("GetIQ")
        # motor_0.getIQ()
        
        # motor_0.getMotorSpecs()
        # motor_0.getCurrentParams()
        # motor_0.getPositionParams()
        # print(motor_0.mode, motor_1.mode,
        #       motor_0.motor_position_target, motor_1.motor_position_target,
        #       motor_0.motor_position_measured, motor_1.motor_position_measured)
        # print(motor_0.params["velocity_measured"], motor_1.params["velocity_measured"])
        # print(motor_0.mode, motor_0.params["position_measured"], motor_0.params["position_target"])
        print(motor_0.params["position_measured"], motor_1.params["position_measured"])

        # print(motor_0.motor_iq_measured, motor_0.motor_iq_target, motor_0.motor_position_measured)
        # print(motor_0.motor_pole_pairs, motor_0.motor_kv)
        # print(motor_0.motor.current_kp, motor_0.motor.current_ki)
        # print(motor_0.motor.position_kp, motor_0.motor.position_ki, motor_0.motor.position_kd)

        motor_0.feed()
        motor_1.feed()

        time.sleep(0.01)
        
        #print(time.time() - t)

if __name__ == "__main__":
    main()
    #motor_1.setMode(RecoilMotorController.MODE_CALIBRATION)