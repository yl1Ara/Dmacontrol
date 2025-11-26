import serial
import numpy as np
import time
import csv
import os
import datetime

import nidaqmx
from simple_pid import PID
from nidaqmx.constants import AcquisitionType, FrequencyUnits, Level, Signal


fmt = '%Y %m %d %H %M %S %z'
new_line = b'\n'
log_dir = 'logfiles'

######Variables######
#cpc_port = 'COM3' 
HV_MAX = 10000.0

flowmeter_port = 'COM5'
flowmeter_baud = 38400

sheath_device = 'Dev2'
sheath_device_counter = 'Ctr0'
sheath_device_output = f'/{sheath_device}/PFI4'
sheath_freq = 200
sheath_current_ds = 0.9
sheath_task = None



sheath_flow = 14
sheath_error_margin = 0.5

pid = PID(0.001, 0.004, 0, setpoint=sheath_flow)  
pid.sample_time = 0.2  
pid.output_limits = (0.01, 0.99)  



sleep_time = 5  # seconds
meas_time = 10  # seconds per size

size_list_nm = [10, 15, 20, 25]

def cpc_read(cpc_port):
    cpc_con = None

    #https://www.manualslib.com/manual/1562828/Airmodus-A11.html?page=71#manual
    try:
        with serial.Serial(cpc_port, 115200, timeout=1) as ser_cpc:
            ser_cpc.write(b':MEAS:OPC\r') #Read OPC pulse duration (total dead time during averaging time), number of pulses,concentration, averaging time and DC compensation state
            return ser_cpc.read_until(new_line).decode('utf-8').strip().split(',')[2]
    except:
        print("CPC Error")
        pass
    return cpc_con

def cunningham_correction(dp, T=293.15, P=101325):
    lambda_0 = 65e-9
    T0 = 273.15
    P0 = 101325
    lambda_air = lambda_0 * (T / T0) * (P0 / P)
    return 1 + (2 * lambda_air / dp) * (1.257 + 0.4 * np.exp(-1.1 * dp / (2 * lambda_air)))

def voltage_from_size(dp_nm, Q_sh_lpm=14.0, T_C=20.0, P=101325, debug=False):
    mu = 1.81e-5    
    e = 1.602e-19    
    r1 = 0.025       
    r2 = 0.033       
    L = 0.28       

    if dp_nm <= 0:
        return 0.0

    dp = dp_nm * 1e-9       
    Q_sh = Q_sh_lpm / 60000  
    ln_r = np.log(r2 / r1)
    T_K = T_C + 273.15

    Cc = cunningham_correction(dp, T=T_K, P=P)

    V = (3 * mu * Q_sh * ln_r * dp) / (2 * L * e * Cc)
    analog = V / (HV_MAX / 10.0) * ((20000 + 470)/20000)
    analog = np.clip(analog, 0.0, 10.0)

    if debug:   
        print(f"dp: {dp_nm} nm, Cc: {Cc:.3f}, HV: {V:.1f} V, analog: {analog:.3f} V")

    return analog

def set_daq_voltage(device_name, voltage):
    global daq_task
    if not device_name or device_name == "None":
        return None

    try:
        voltage = float(voltage)
    except (TypeError, ValueError):
        print(f"Invalid voltage: {voltage}")
        return None

    if "daq_task" in globals() and daq_task is not None:
        try:
            daq_task.stop()
        except Exception:
            pass
        try:
            daq_task.close()
        except Exception:
            pass
        daq_task = None

    try:
        daq_task = nidaqmx.Task()
        daq_task.ao_channels.add_ao_voltage_chan(f"{device_name}/ao0")
        daq_task.write(voltage)
    except Exception as e:
        print(f"DAQ Error: {e}")
        if "daq_task" in globals() and daq_task is not None:
            try:
                daq_task.close()
            except Exception:
                pass
            daq_task = None
        return None

    return daq_task

def read_ai_voltage(device_name, channel_name):
    try:
        with nidaqmx.Task() as ai_task:
            ai_task.ai_channels.add_ai_voltage_chan(f"{device_name}/{channel_name}")
            voltage = ai_task.read()/(1/470)
            return voltage
    except Exception as e:
        print(f"Error reading AI voltage: {e}")
        return None

    
def set_sheath_duty(duty_cycle):
    global sheath_task, sheath_current_ds, sheath_device, sheath_device_counter, sheath_device_output
    ds = float(duty_cycle)

    if sheath_task:
        try:
            sheath_task.stop()
            sheath_task.close()
        except Exception as e:
            print(f"Error stopping/closing previous task: {e}")

    task = nidaqmx.Task()
    task.co_channels.add_co_pulse_chan_freq(
        f"{sheath_device}/{sheath_device_counter}",
        units=FrequencyUnits.HZ,
        idle_state=Level.LOW,
        initial_delay=1e-6,
        freq=sheath_freq,
        duty_cycle=ds,
    )
    task.timing.cfg_implicit_timing(sample_mode=AcquisitionType.CONTINUOUS)
    task.export_signals.export_signal(
        Signal.COUNTER_OUTPUT_EVENT, sheath_device_output
    )
    task.start()

    sheath_task = task
    sheath_current_ds = duty_cycle




def read_sheath_flow_mbed(flowmeter_port=flowmeter_port, flowmeter_baud=flowmeter_baud):
    flow_query = b'DAFTP0001\r' #DATA,ASCII,FLOW,Temp=T/x, pressure=P/x,sample size=nnnn
    try:
        with serial.Serial(flowmeter_port, flowmeter_baud, timeout=0.2) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.write(flow_query)
            line = ser.read_until(b'\n').decode('ascii', errors='ignore').strip()
            flow,temp,press= ser.read_until(b"\n").decode("ascii", errors="ignore").strip().split(',')
            return flow, temp, press
    except Exception as e:
        print(f"TSI flowmeter error: {e}")
        return None

def set_sheath_flow(sheath):
    global pid

    try:
        setpoint = float(sheath)
    except (TypeError, ValueError):
        print(f"Invalid sheath setpoint: {sheath}")
        return None

    pid.setpoint = setpoint

    flow, temp, press = map(float, read_sheath_flow_mbed())
    
    if flow is None:
        return None

    duty = 1-pid(flow)

    try:
        set_sheath_duty(duty)
    except Exception as e:
        print(f"DAQ PWM error: {e}")
        return None

    return duty

def get_log_filenames():
    
    timestamp = datetime.datetime.now().strftime('%Y%m%d')
    return (
        os.path.join(log_dir, f'orbi_inlet_{timestamp}.csv')
    )

#Example measurement loop but not properly implemented so use gui.py
def measurement_loop():
    os.makedirs(log_dir, exist_ok=True)
    
    set_sheath_flow(sheath_flow)

    file_exists = os.path.exists(get_log_filenames())
    while True:
        for dp in size_list_nm:
            cpc_count = cpc_read()
            sheath = read_sheath_flow_mbed() or sheath_flow
            analog_voltage = voltage_from_size(dp, Q_sh_lpm=sheath, T_C=20.0, debug=True)
            set_daq_voltage("Dev1", analog_voltage)
            try:
                if abs(float(sheath) - sheath_flow) > sheath_error_margin:
                    set_sheath_flow(sheath_flow)
                    print(f"Corrected sheath flow to {sheath_flow} LPM")
            except:
                print("Sheath flow read error.")
                pass
            row = {'time': time.strftime(fmt), 'size_nm': dp, 'analog_voltage': analog_voltage, 'cpc_count': cpc_count, 'sheath_flow': sheath}

            with open(get_log_filenames(), 'a', newline='') as logfile:
                writer = csv.writer(logfile)
                if not file_exists:
                    writer.writerow(row.keys())
                    file_exists = True
                writer.writerow(row.values())
            time.sleep(sleep_time)

if __name__ == "__main__":
    #flow, temp, press = read_sheath_flow_mbed(flowmeter_baud=38400, flowmeter_port='COM5')
    #set_sheath_flow(14)
    '''while True:
        flow, temp, press = read_sheath_flow_mbed(flowmeter_baud=38400, flowmeter_port='COM5')
        set_sheath_flow(14)'''
    '''while True:
        cpc_count = cpc_read('COM9')
        time.sleep(5)'''
    set_daq_voltage("Dev2", 0.1)