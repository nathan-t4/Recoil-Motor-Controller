class MotorParams():
    def __init__(self):
        self.position_measured = 0
        self.velocity_measured = 0
        self.torque_measured = 0
        self.iq_measured = 0
        self.id_measured = 0
        self.vq_measured = 0
        self.vd_measured = 0

        self.position_target = 0
        self.torque_target = 0
        self.iq_target = 0
        self.id_target = 0
        self.vq_target = 0
        self.vd_target = 0

        self.position_kp = 0
        self.position_ki = 0
        self.position_kd = 0

        self.current_kp = 0
        self.current_ki = 0

        self.v_bus = 0

        self.pole_pairs = 0
        self.kv = 0