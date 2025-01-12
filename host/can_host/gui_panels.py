import tkinter as tk
import tkinter.ttk as ttk

def createSectionControl(panel, esc, can_protocol):
    button_idle_style = ttk.Style()
    button_idle_style.configure("Idle.TButton", foreground="#000000")
    button_calibrate_style = ttk.Style()
    button_calibrate_style.configure("Calibrate.TButton", foreground="#666666")
    button_torque_style = ttk.Style()
    button_torque_style.configure("Torque.TButton", foreground="#666666")
    button_velocity_style = ttk.Style()
    button_velocity_style.configure("Velocity.TButton", foreground="#666666")
    button_position_style = ttk.Style()
    button_position_style.configure("Position.TButton", foreground="#666666")
    button_open_idq_style = ttk.Style()
    button_open_idq_style.configure("OpenIdq.TButton", foreground="#666666")

    def setMode(mode):
        if mode == can_protocol.RecoilMotorController.MODE_IDLE:
            button_idle_style.configure("Idle.TButton", foreground="#000000")
            button_calibrate_style.configure("Calibrate.TButton", foreground="#666666")
            button_torque_style.configure("Torque.TButton", foreground="#666666")
            button_velocity_style.configure("Velocity.TButton", foreground="#666666")
            button_position_style.configure("Position.TButton", foreground="#666666")
            button_open_idq_style.configure("OpenIdq.TButton", foreground="#666666")
            esc.commandMode(can_protocol.RecoilMotorController.MODE_IDLE)

        elif mode == can_protocol.RecoilMotorController.MODE_CALIBRATION:
            button_idle_style.configure("Idle.TButton", foreground="#666666")
            button_calibrate_style.configure("Calibrate.TButton", foreground="#000000")
            button_torque_style.configure("Torque.TButton", foreground="#666666")
            button_velocity_style.configure("Velocity.TButton", foreground="#666666")
            button_position_style.configure("Position.TButton", foreground="#666666")
            button_open_idq_style.configure("OpenIdq.TButton", foreground="#666666")
            esc.commandMode(can_protocol.RecoilMotorController.MODE_CALIBRATION)
        elif mode == can_protocol.RecoilMotorController.MODE_TORQUE:
            button_idle_style.configure("Idle.TButton", foreground="#666666")
            button_calibrate_style.configure("Calibrate.TButton", foreground="#666666")
            button_torque_style.configure("Torque.TButton", foreground="#000000")
            button_velocity_style.configure("Velocity.TButton", foreground="#666666")
            button_position_style.configure("Position.TButton", foreground="#666666")
            button_open_idq_style.configure("OpenIdq.TButton", foreground="#666666")
            esc.commandMode(can_protocol.RecoilMotorController.MODE_TORQUE)
        elif mode == can_protocol.RecoilMotorController.MODE_VELOCITY:
            button_idle_style.configure("Idle.TButton", foreground="#666666")
            button_calibrate_style.configure("Calibrate.TButton", foreground="#666666")
            button_torque_style.configure("Torque.TButton", foreground="#666666")
            button_velocity_style.configure("Velocity.TButton", foreground="#000000")
            button_position_style.configure("Position.TButton", foreground="#666666")
            button_open_idq_style.configure("OpenIdq.TButton", foreground="#666666")
            esc.commandMode(can_protocol.RecoilMotorController.MODE_VELOCITY)
        elif mode == can_protocol.RecoilMotorController.MODE_POSITION:
            button_idle_style.configure("Idle.TButton", foreground="#666666")
            button_calibrate_style.configure("Calibrate.TButton", foreground="#666666")
            button_torque_style.configure("Torque.TButton", foreground="#666666")
            button_velocity_style.configure("Velocity.TButton", foreground="#666666")
            button_position_style.configure("Position.TButton", foreground="#000000")
            button_open_idq_style.configure("OpenIdq.TButton", foreground="#666666")
            esc.commandMode(can_protocol.RecoilMotorController.MODE_POSITION)
        elif mode == can_protocol.RecoilMotorController.MODE_OPEN_IDQ:
            button_idle_style.configure("Idle.TButton", foreground="#666666")
            button_calibrate_style.configure("Calibrate.TButton", foreground="#666666")
            button_torque_style.configure("Torque.TButton", foreground="#666666")
            button_velocity_style.configure("Velocity.TButton", foreground="#666666")
            button_position_style.configure("Position.TButton", foreground="#666666")
            button_open_idq_style.configure("OpenIdq.TButton", foreground="#000000")
            esc.commandMode(can_protocol.RecoilMotorController.MODE_OPEN_IDQ)

    button_idle = ttk.Button(panel, text="IDLE Mode", style="Idle.TButton", command=lambda: setMode(can_protocol.RecoilMotorController.MODE_IDLE))
    button_calibrate = ttk.Button(panel, text="CALIBRATE Mode", style="Calibrate.TButton", command=lambda: setMode(can_protocol.RecoilMotorController.MODE_CALIBRATION))
    button_torque = ttk.Button(panel, text="TORQUE Mode", style="Torque.TButton", command=lambda: setMode(can_protocol.RecoilMotorController.MODE_TORQUE))
    button_velocity = ttk.Button(panel, text="VELOCITY Mode", style="Velocity.TButton", command=lambda: setMode(can_protocol.RecoilMotorController.MODE_VELOCITY))
    button_position = ttk.Button(panel, text="POSITION Mode", style="Position.TButton", command=lambda: setMode(can_protocol.RecoilMotorController.MODE_POSITION))
    button_open_idq = ttk.Button(panel, text="OPEN IDQ Mode", style="OpenIdq.TButton", command=lambda: setMode(can_protocol.RecoilMotorController.MODE_OPEN_IDQ))
    button_idle.grid(column=0, row=0, sticky=(tk.W, tk.E))
    button_calibrate.grid(column=0, row=1, sticky=(tk.W, tk.E))
    button_torque.grid(column=0, row=2, sticky=(tk.W, tk.E))
    button_velocity.grid(column=0, row=3, sticky=(tk.W, tk.E))
    button_position.grid(column=0, row=4, sticky=(tk.W, tk.E))
    button_open_idq.grid(column=0, row=5, sticky=(tk.W, tk.E))

def createTargetControl(panel, esc):
    button_step_target = ttk.Button(panel, text="Step", command=lambda:esc.setTargetFcn("Step"))
    button_step_target.grid(column=0, row=1)
    button_neg_step_target = ttk.Button(panel, text="-Step", command=lambda:esc.setTargetFcn("Neg_Step"))
    button_neg_step_target.grid(column=1, row=1)
    button_sin2w_target = ttk.Button(panel, text="sin(2w)", command=lambda:esc.setTargetFcn("sin2w"))
    button_sin2w_target.grid(column=2, row=1)
    button_sin4w_target = ttk.Button(panel, text="sin(4w)", command=lambda:esc.setTargetFcn("sin4w"))
    button_sin4w_target.grid(column=3, row=1)
    button_sin8w_target = ttk.Button(panel, text="sin(8w)", command=lambda:esc.setTargetFcn("sin8w"))
    button_sin8w_target.grid(column=4, row=1)
    button_rect_target = ttk.Button(panel, text="rect", command=lambda:esc.setTargetFcn("rect"))
    button_rect_target.grid(column=5, row=1)

def createParamsControl(panel, esc):
    column = 0
    row = 0
    def createEntry(var_name):
        # TODO
        global column
        input = tk.StringVar()
        label = ttk.Label(panel, text=var_name)
        label.grid(column, row, sticky=(tk.W, tk.E))
        set_param = ttk.Entry(panel, width=15, textvariable=input)
        column = column+1
        set_param.grid(column, row)
        return set_param, input.get()

    # set_current_kp, current_kp = createEntry("Current Kp")
    current_kp = tk.StringVar()
    label_current_kp = ttk.Label(panel, text="Current Kp")
    label_current_kp.grid(column=0, row=0, sticky=(tk.W, tk.E))
    set_current_kp = ttk.Entry(panel, width=15, textvariable=current_kp)
    set_current_kp.bind(sequence='<Return>', func=lambda _:esc.accessParam({'name':'CurrentParams','current_kp':current_kp.get(),'current_ki':None,'operation':'set'}))
    set_current_kp.grid(column=1, row=0, sticky=(tk.W, tk.E))

    current_ki = tk.StringVar()
    label_current_ki = ttk.Label(panel, text="Current Ki")
    label_current_ki.grid(column=0, row=1, sticky=(tk.W, tk.E))
    set_current_ki = ttk.Entry(panel, width=15, textvariable=current_ki)
    set_current_ki.grid(column=1, row=1, sticky=(tk.W, tk.E))
    set_current_ki.bind('<Return>', func=lambda _:esc.accessParam({'name':'CurrentParams','current_kp':None,'current_ki':current_ki.get(),'operation':'set'}))

    position_kp = tk.StringVar()
    label_position_kp = ttk.Label(panel, text="Position Kp")
    label_position_kp.grid(column=0, row=2, sticky=(tk.W, tk.E))
    set_position_kp = ttk.Entry(panel, width=15, textvariable=position_kp)
    set_position_kp.grid(column=1, row=2, sticky=(tk.W, tk.E))
    set_position_kp.bind(sequence='<Return>', func=lambda _:esc.accessParam({'name':'PositionParams','position_kp':position_kp.get(),'position_kd':None,'operation':'set'}))

    # position_ki = tk.StringVar()
    label_position_ki = ttk.Label(panel, text="Position Ki")
    label_position_ki.grid(column=0, row=3, sticky=(tk.W, tk.E))
    # set_position_ki = ttk.Entry(panel, width=15, textvariable=position_ki)
    # set_position_ki.grid(column=1, row=3, sticky=(tk.W, tk.E))
    # set_position_ki.bind(sequence='<Return>', func=lambda _:esc.accessParam({'name':'PositionParams','position_ki':position_ki.get(),'operation':'set'}))

    position_kd = tk.StringVar()
    label_position_kd = ttk.Label(panel, text="Position Kd")
    label_position_kd.grid(column=0, row=4, sticky=(tk.W, tk.E))
    set_position_kd = ttk.Entry(panel, width=15, textvariable=position_kd)
    set_position_kd.grid(column=1, row=4, sticky=(tk.W, tk.E))
    set_position_kd.bind(sequence='<Return>', func=lambda _:esc.accessParam({'name':'PositionParams','position_kp':None,'position_kd':position_kd.get(),'operation':'set'}))

    position_limit_lower = tk.StringVar()
    label_position_limit_lower = ttk.Label(panel, text="Position Limit Lower")
    label_position_limit_lower.grid(column=0, row=5, sticky=(tk.W, tk.E))
    set_position_limit_lower = ttk.Entry(panel, width=15, textvariable=position_limit_lower)
    set_position_limit_lower.grid(column=1, row=5, sticky=(tk.W, tk.E))
    set_position_limit_lower.bind(sequence='<Return>', func=lambda _:esc.accessParam({'name':'PositionLimits','position_limit_lower':position_limit_lower.get(),'position_limit_upper':None,'operation':'set'}))

    position_limit_upper = tk.StringVar()
    label_position_limit_upper = ttk.Label(panel, text="Position Limit Upper")
    label_position_limit_upper.grid(column=0, row=6, sticky=(tk.W, tk.E))
    set_position_limit_upper = ttk.Entry(panel, width=15, textvariable=position_limit_upper)
    set_position_limit_upper.grid(column=1, row=6, sticky=(tk.W, tk.E))
    set_position_limit_upper.bind(sequence='<Return>', func=lambda _:esc.accessParam({'name':'PositionLimits','position_limit_lower':None,'position_limit_upper':position_limit_upper.get(),'operation':'set'}))

    velocity_limit_lower = tk.StringVar()
    label_velocity_limit_lower = ttk.Label(panel, text="Velocity Limit Lower")
    label_velocity_limit_lower.grid(column=0, row=7, sticky=(tk.W, tk.E))
    set_velocity_limit_lower = ttk.Entry(panel, width=15, textvariable=velocity_limit_lower)
    set_velocity_limit_lower.grid(column=1, row=7, sticky=(tk.W, tk.E))
    set_velocity_limit_lower.bind(sequence='<Return>', func=lambda _:esc.accessParam({'name':'VelocityLimits','velocity_limit_lower':velocity_limit_lower.get(),'velocity_limit_upper':None,'operation':'set'}))

    velocity_limit_upper = tk.StringVar()
    label_velocity_limit_upper = ttk.Label(panel, text="Velocity Limit Upper")
    label_velocity_limit_upper.grid(column=0, row=8, sticky=(tk.W, tk.E))
    set_velocity_limit_upper = ttk.Entry(panel, width=15, textvariable=velocity_limit_upper)
    set_velocity_limit_upper.grid(column=1, row=8, sticky=(tk.W, tk.E))
    set_velocity_limit_upper.bind(sequence='<Return>', func=lambda _:esc.accessParam({'name':'VelocityLimits','velocity_limit_lower':None,'velocity_limit_upper':velocity_limit_upper.get(),'operation':'set'}))

    torque_limit_lower = tk.StringVar()
    label_torque_limit_lower = ttk.Label(panel, text="Torque Limit Lower")
    label_torque_limit_lower.grid(column=0, row=9, sticky=(tk.W, tk.E))
    set_torque_limit_lower = ttk.Entry(panel, width=15, textvariable=torque_limit_lower)
    set_torque_limit_lower.grid(column=1, row=9, sticky=(tk.W, tk.E))
    set_torque_limit_lower.bind(sequence='<Return>', func=lambda _:esc.accessParam({'name':'TorqueLimits','torque_limit_lower':torque_limit_lower.get(),'torque_limit_upper':None,'operation':'set'}))

    torque_limit_upper = tk.StringVar()
    label_torque_limit_upper = ttk.Label(panel, text="Torque Limit Upper")
    label_torque_limit_upper.grid(column=0, row=10, sticky=(tk.W, tk.E))
    set_torque_limit_upper = ttk.Entry(panel, width=15, textvariable=torque_limit_upper)
    set_torque_limit_upper.grid(column=1, row=10, sticky=(tk.W, tk.E))
    set_torque_limit_upper.bind(sequence='<Return>', func=lambda _:esc.accessParam({'name':'TorqueLimits','torque_limit_lower':None,'torque_limit_upper':torque_limit_upper.get(),'operation':'set'}))

    button_get_current_params = ttk.Button(panel, text="Current Params", width=15, command=lambda:esc.accessParam({'name':'CurrentParams','current_kp':None,'current_ki':None}))
    button_get_current_params.grid(column=0, row=11)

    button_get_position_params = ttk.Button(panel, text="Position Params", width=15, command=lambda:esc.accessParam({'name':'PositionParams','position_kp':None,'position_kd':None}))
    button_get_position_params.grid(column=0, row=12)

    button_get_position_limits = ttk.Button(panel, text="Position Limits", width=15, command=lambda:esc.accessParam({'name':'PositionLimits','position_limit_lower':None,'position_limit_upper':None}))
    button_get_position_limits.grid(column=0, row=13)

    button_get_torque_limits = ttk.Button(panel, text="Torque Limits", width=15, command=lambda:esc.accessParam({'name':'TorqueLimits','torque_limit_lower':None,'torque_limit_upper':None}))
    button_get_torque_limits.grid(column=0, row=14)

    button_get_velocity_limits = ttk.Button(panel, text="Velocity Limits", width=15, command=lambda:esc.accessParam({'name':'VelocityLimits','velocity_limit_lower':None,'velocity_limit_upper':None}))
    button_get_velocity_limits.grid(column=0, row=15)

    button_save_params = ttk.Button(panel, text="Save to Flash", width=15, command=lambda:esc.saveToFlash())
    button_save_params.grid(column=1, row=11)