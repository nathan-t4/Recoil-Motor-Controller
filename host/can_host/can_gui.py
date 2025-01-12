import time

import tkinter as tk
import tkinter.ttk as ttk

import serial
import math
import can_protocol
import motor_can
import gui_panels

FPS = 10

display_mode = "GENERAL"
device_id = 2

class Plot(ttk.Frame):
    def __init__(self, parent, traces, name="plot", x_pkpk=[0, 100], y_pkpk=[-1, 1]):
        super().__init__(parent)
        self.parent = parent
        self.label = ttk.Label(self, text=name)
        self.label.grid(column=0, row=0)
        self.canvas = tk.Canvas(self, width=600, height=160, background='gray75')
        self.canvas.grid(column=0, row=1, sticky=(tk.N, tk.W, tk.E, tk.S))

        self.x_pkpk = x_pkpk
        self.y_pkpk = y_pkpk
        self.x_range = self.x_pkpk[1] - self.x_pkpk[0]
        self.y_range = self.y_pkpk[1] - self.y_pkpk[0]
        self.ys = {}
        self.y_colors = {}

        self.x_counter = 0

        for trace in traces:
            self.ys[trace["name"]] = [0.] * self.x_range
            self.y_colors[trace["name"]] = trace["color"] if trace["color"] else "#333333"
        
        
    def updateData(self, data):
        for key in self.ys:
            val = data.get(key)
            self.ys[key].append(val if val else 0)
            self.ys[key].pop(0)
        self.x_counter += 1

    def update(self):            
        self.h_scale = self.canvas.winfo_width() / self.x_range
        self.v_scale = self.canvas.winfo_height() / self.y_range

        self.x_counter = self.x_counter % self.x_range
        x_counter_half = (self.x_counter + .5 * self.x_range) % self.x_range

        self.canvas.delete("all")
        self.canvas.create_line(0, self.canvas.winfo_height() - (-self.y_pkpk[0]) * self.v_scale, self.canvas.winfo_width(), self.canvas.winfo_height()-(-self.y_pkpk[0]) * self.v_scale, fill="#666666")
        self.canvas.create_line(self.canvas.winfo_width() - self.x_counter * self.h_scale, 0, self.canvas.winfo_width() - self.x_counter * self.h_scale, self.canvas.winfo_height(), fill="#666666")
        self.canvas.create_line(self.canvas.winfo_width() - x_counter_half * self.h_scale, 0, self.canvas.winfo_width() - x_counter_half * self.h_scale, self.canvas.winfo_height(), fill="#999999")
        
        for i in range(self.x_range-1):
            for key in self.ys:
                self.canvas.create_line(
                    i*self.h_scale,
                    self.canvas.winfo_height()-(self.ys[key][i] - self.y_pkpk[0]) * self.v_scale,
                    (i+1)*self.h_scale,
                    self.canvas.winfo_height()-(self.ys[key][i+1] - self.y_pkpk[0]) * self.v_scale,
                    fill=self.y_colors[key],
                    width=1)
            
        self.parent.after(int(1000/FPS), self.update)

esc = motor_can.ESCControl(device_id=device_id)


root = tk.Tk()
root.title(" MyServoGUI")
root.geometry("1000x600")
root.minsize(1000, 600)

section_plot = ttk.Frame(root)
section_plot.grid(column=1, row=0)

section_control = ttk.Frame(root)
section_control.grid(column=2, row=0, sticky=(tk.W, tk.N, tk.E))

params_control = ttk.Frame(root)
params_control.grid(column=0, row=0, sticky=(tk.W, tk.N, tk.E))
params_control['padding'] = (0, 80, 0, 0) # (left, top, right, bottom)

panel_selection = ttk.Frame(section_plot)
panel_selection.grid(column=0, row=0, sticky=(tk.W))
panel_general = ttk.Frame(section_plot)
panel_general.grid(column=0, row=1)
panel_current = ttk.Frame(section_plot)
panel_current.grid(column=0, row=1)
panel_voltage = ttk.Frame(section_plot)
panel_voltage.grid(column=0, row=1)

panel_current.grid_forget()
panel_voltage.grid_forget()

gui_panels.createSectionControl(section_control, esc, can_protocol)
gui_panels.createParamsControl(params_control, esc)
gui_panels.createTargetControl(panel_selection, esc)

def showGeneralPlot():
    global display_mode
    
    panel_general.grid(column=0, row=1)
    panel_current.grid_forget()
    panel_voltage.grid_forget()
    display_mode = "GENERAL"
    
def showCurrentPlot():
    global display_mode
    
    panel_general.grid_forget()
    panel_current.grid(column=0, row=1)
    panel_voltage.grid_forget()
    display_mode = "CURRENT"
    
def showVoltagePlot():
    global display_mode
    
    panel_general.grid_forget()
    panel_current.grid_forget()
    panel_voltage.grid(column=0, row=1)
    display_mode = "VOLTAGE"

button_show_general = ttk.Button(panel_selection, text="General", command=showGeneralPlot)
button_show_general.grid(column=0, row=0)
button_show_current = ttk.Button(panel_selection, text="Current", command=showCurrentPlot)
button_show_current.grid(column=1, row=0)
button_show_voltage = ttk.Button(panel_selection, text="Voltage", command=showVoltagePlot)
button_show_voltage.grid(column=2, row=0)

plot_position = Plot(panel_general,
            traces=[
                {
                    "name": "position_measured",
                    "color": "#00FFFF",
                }, {
                    "name": "position_setpoint",
                    "color": "#FFFF00",
                }],
            name="Position",
            y_pkpk=[-20, 20])
plot_position.grid(column=0, row=0)
plot_velocity = Plot(panel_general,
            traces=[
                {
                    "name": "velocity_measured",
                    "color": "#00FFFF",
                }, {
                    "name": "velocity_setpoint",
                    "color": "#FFFF00",
                }],
            name="Velocity",
            y_pkpk=[-100, 100])
plot_velocity.grid(column=0, row=1)
plot_torque = Plot(panel_general,
            traces=[
                {
                    "name": "torque_measured",
                    "color": "#00FFFF",
                }, {
                    "name": "torque_setpoint",
                    "color": "#FFFF00",
                }],
            name="Torque",
            y_pkpk=[-5, 5])
plot_torque.grid(column=0, row=2)

plot_phase_current = Plot(panel_current,
            traces=[
                {
                    "name": "i_a",
                    "color": "#FF0000",
                }, {
                    "name": "i_b",
                    "color": "#00FF00",
                }, {
                    "name": "i_c",
                    "color": "#0000FF",
                }],
            name="Phase Current",
            y_pkpk=[-1, 1])
plot_phase_current.grid(column=0, row=0)
plot_alphabeta_current = Plot(panel_current,
            traces=[
                {
                    "name": "i_alpha",
                    "color": "#FFFF00",
                }, {
                    "name": "i_beta",
                    "color": "#00FFFF",
                }],
            name="α β Current",
            y_pkpk=[-1, 1])
plot_alphabeta_current.grid(column=0, row=1)
plot_q_current = Plot(panel_current,
            traces=[
                {
                    "name": "i_q_measured",
                    "color": "#FFFF00",
                }, {
                    "name": "i_q_target",
                    "color": "#00FFFF",
                }],
            name="Q-Axis Current",
            y_pkpk=[-5, 5])
plot_q_current.grid(column=0, row=2)

plot_bus_voltage = Plot(panel_voltage,
            traces=[
                {
                    "name": "v_bus",
                    "color": "#FF0000",
                }],
            name="Bus Voltage",
            y_pkpk=[-0.1, 15])
plot_bus_voltage.grid(column=0, row=0)
plot_phase_voltage = Plot(panel_voltage,
            traces=[
                {
                    "name": "v_a",
                    "color": "#FF0000",
                },
                {
                    "name": "v_b",
                    "color": "#00FF00",
                },
                {
                    "name": "v_c",
                    "color": "#0000FF",
                }],
            name="Phase Voltage",
            y_pkpk=[-10, 10])
plot_phase_voltage.grid(column=0, row=1)


def receivePacket():
    global display_mode

    esc.motor.ping()
    esc.motor.getMode()
    esc.motor.getCurrentParams()
    esc.motor.getPositionParams()
    esc.motor.getGeneral()
    # print(f"Kp: {esc.motor.params['current_kp'] Ki: {esc.motor.params['current_ki']}"")

    esc.setTarget()

    # print(esc.motor.params['position_measured'], esc.motor.params['position_target'])
    # print(esc.motor.params['velocity_measured'], esc.motor.params['velocity_target'])
    # print(esc.motor.params['iq_measured,'] esc.motor.params['iq_target'])
    if display_mode == "GENERAL":
        esc.updateGeneral()
        plot_position.updateData({
            "position_measured": esc.motor.params['position_measured'],
            "position_setpoint": esc.motor.params['position_target'],
            })
        plot_velocity.updateData({
            "velocity_measured": esc.motor.params['velocity_measured'],
            "velocity_setpoint": esc.motor.params['velocity_target']
            })
        plot_torque.updateData({
            "torque_measured": esc.motor.params['torque_measured'],
            "torque_setpoint": esc.motor.params['torque_target'],
            })
    
    elif display_mode == "CURRENT":
        esc.updateCurrent()
        plot_phase_current.updateData({
            "i_a": 0,
            "i_b": 0,
            "i_c": 0
            })
        plot_alphabeta_current.updateData({
            "i_alpha": 0,
            "i_beta": 0
            })
        plot_q_current.updateData({
            "i_q_target": esc.motor.params['iq_target'],
            "i_q_measured": esc.motor.params['iq_measured']
            })
    
    elif display_mode == "VOLTAGE":
        esc.updateVoltage()
        plot_bus_voltage.updateData({
            "v_bus": esc.motor.params['v_bus']
            })
        plot_phase_voltage.updateData({
            "v_a": 0,
            "v_b": 0,
            "v_c": 0
            })

    esc.motor.feed()
    root.after(1, receivePacket)

root.after(1, receivePacket)

plot_position.update()
plot_velocity.update()
plot_torque.update()
                     
plot_phase_current.update()
plot_alphabeta_current.update()
plot_q_current.update()

plot_bus_voltage.update()
plot_phase_voltage.update()


root.mainloop()