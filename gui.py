import panel as pn
import pandas as pd
import time
import control as ctl
import numpy as np
import os
import csv
import plotly.express as px


#Run with (in terminal):
#Set-ExecutionPolicy Unrestricted -Scope Process
#.\.venv\Scripts\activate
#panel serve gui.py --autoreload --show

pn.extension("plotly")

#### Widgets ####
cpc_com_port = pn.widgets.TextInput(name="CPC COM port", value="COM9")
nidaq_device = pn.widgets.TextInput(name="NI DAQ Device", value="Dev2")
flowmeter_com_port = pn.widgets.TextInput(name="Flowmeter COM port", value="COM5")

measured_voltage = 0

start_button = pn.widgets.Toggle(name="Start measurement", button_type="success")
sheath_slider = pn.widgets.IntInput(name="Sheath flow setpoint (L/min)", value=int(ctl.sheath_flow), step=1)
size_selector = pn.widgets.ArrayInput(
    name="Sizes (nm)",
    value=np.array([10, 15, 20, 25]),  
    max_array_size=1000,              
    placeholder="[10, 15, 20, 25]",
)

meas_time = pn.widgets.IntInput(name="Measurement time per size (s)", value=int(ctl.meas_time), step=1)
sleep_time = pn.widgets.IntInput(name="Sleep time between measurements (s)", value=int(ctl.sleep_time), step=1)

status_text = pn.pane.Markdown("Status: idle")
last_row_pane = pn.pane.Str("Last measurement: -")
table_pane = pn.widgets.DataFrame(pd.DataFrame(columns=["time","size_nm","analog_voltage","cpc_count","sheath_flow"]),
                                  height=200, width=800)


rows = []
current_size_index = 0
phase = "idle"  
phase_start_time = time.time()

def get_sizes():
    try:
        arr = np.array(size_selector.value).ravel()
        arr = arr[np.isfinite(arr)]
        arr = arr[arr > 0]
        return [int(x) for x in arr]
    except Exception as e:
        status_text.object = f"Size parse error: {e}"
        return []

def parse_sheath_value(raw):
    if raw is None:
        return None
    try:
        return float(str(raw).split()[0])
    except ValueError:
        return None

def measurement_step():
    global current_size_index, phase, phase_start_time

    if not start_button.value:
        return  # not running

    sizes = get_sizes()
    if not sizes:
        status_text.object = "Status: no sizes defined"
        return

    now = time.time()
    meas_sec = float(meas_time.value)
    sleep_sec = float(sleep_time.value)

    # --- start state ---
    if phase == "idle":
        # move to first size
        phase = "measuring"
        phase_start_time = now
        current_size_index = 0

        ctl.set_sheath_flow(float(sheath_slider.value))
        dp = sizes[current_size_index]
        # set voltage for this size once at start
        flow, temp, press = ctl.read_sheath_flow_mbed(flowmeter_baud=38400, flowmeter_port=flowmeter_com_port.value)
        sheath_val = parse_sheath_value(flow) or float(sheath_slider.value)
        analog_voltage = ctl.voltage_from_size(dp, Q_sh_lpm=sheath_val, P=float(press)*1000, debug=False)
        ctl.set_daq_voltage(nidaq_device.value, analog_voltage)
        measured_voltage = ctl.read_ai_voltage(nidaq_device.value, "ai0")

    # --- measuring phase ---
    if phase == "measuring":
        dp = sizes[current_size_index]

        # do one measurement sample
        cpc_count = ctl.cpc_read(cpc_port=cpc_com_port.value)
        flow, temp, press = map(float, ctl.read_sheath_flow_mbed(flowmeter_baud=38400, flowmeter_port=flowmeter_com_port.value))
        sheath_val = parse_sheath_value(flow) or float(sheath_slider.value)

        
        ctl.set_sheath_flow(float(sheath_slider.value))

        analog_voltage = ctl.voltage_from_size(dp, Q_sh_lpm=sheath_val, P=float(press)*1000, debug=False)
        ctl.set_daq_voltage(nidaq_device.value, analog_voltage)
        #measured_voltage = ctl.read_ai_voltage(nidaq_device.value, "ai0")

        row = {
            "time": time.strftime(ctl.fmt),
            "size_nm": dp,
            "cpc_count": cpc_count,
            "sheath_flow": flow,
        }
        rows.append(row)
        last_row_pane.object = str(row)
        fname = ctl.get_log_filenames()

        os.makedirs(os.path.dirname(fname), exist_ok=True)

        file_exists = os.path.exists(fname)
        with open(fname, "a", newline="") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(row.keys())
            writer.writerow(row.values())

        fname = ctl.get_log_filenames()
        file_exists = os.path.exists(fname)
        with open(fname, "a", newline="") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(row.keys())
            writer.writerow(row.values())


        if now - phase_start_time >= meas_sec:
            phase_start_time = now
            current_size_index += 1
            if current_size_index >= len(sizes):
                current_size_index = 0
            



    if rows:
        df = pd.DataFrame(rows[-100:])
        table_pane.value = df



callback = pn.state.add_periodic_callback(measurement_step, period=int(sleep_time.value) * 1000, start=True)

def on_sleep_change(event):
    new_period = max(100, int(event.new) * 1000)
    callback.period = new_period

sleep_time.param.watch(on_sleep_change, "value")

def on_start_change(event):
    global phase, phase_start_time, current_size_index
    if event.new:
        status_text.object = "Status: running"
        phase = "idle"
        phase_start_time = time.time()
        current_size_index = 0
    else:
        status_text.object = "Status: stopped"

from plotly.subplots import make_subplots

def make_plot(df):
    if df is None or df.empty:
        return pn.pane.Markdown("No data")
    
    hover_strings = [
    " ".join(t.split(" ")[-4:-1])
    for t in df["time"]
    ]   


    fig = make_subplots(
        rows=2, cols=1,
        shared_xaxes=True,
        row_heights=[0.20, 0.80],  # top size, bottom cpc
        vertical_spacing=0.05
    )

    fig.add_scatter(
        x=df["time"],
        y=df["size_nm"],
        mode="lines",
        name="Particle Size (nm)",
        customdata=hover_strings,
        hovertemplate="Size: %{y}<br>Time: %{customdata}<extra></extra>",
        row=1, col=1
    )

    fig.add_scatter(
        x=df["time"],
        y=df["cpc_count"].astype(float),
        mode="lines",
        name="CPC Concentration (#/cm³)",
        customdata=hover_strings,
        hovertemplate="Conc: %{y}<br>Time: %{customdata}<extra></extra>",
        row=2, col=1
    )

    fig.update_yaxes(title_text="Size (nm)", row=1, col=1)
    fig.update_yaxes(title_text="CPC (#/cm³)", row=2, col=1)
    fig.update_xaxes(title_text="Time", row=2, col=1)

    fig.update_layout(
        title="Live CPC & Particle Size",
        margin=dict(l=20, r=20, t=40, b=20),
        height=500,
        width=1500,
        showlegend=True,
    )

    return pn.pane.Plotly(fig, config={"responsive": True})

plot_pane = pn.bind(make_plot, table_pane.param.value)


start_button.param.watch(on_start_change, 'value')

#### Layout ####
layout = pn.Column(
    "# DMA / CPC Control GUI",
    pn.Row(cpc_com_port, nidaq_device, flowmeter_com_port),
    "# CPC / DMA control panel",
    pn.Row(start_button, status_text),
    pn.Row(sheath_slider, size_selector),
    pn.Row(meas_time, sleep_time),
    "### Live data",
    last_row_pane,
    table_pane,
    "### Live plot",
    plot_pane,
)


layout.servable()
