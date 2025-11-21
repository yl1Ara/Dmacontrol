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
cpc_port = 'COM3' 
HV_MAX = 5000.0

flowmeter_port = 'COM5'
flowmeter_baud = 38400

sheath_device = 'Dev0'
sheath_device_counter = 'ctr0'
sheath_device_output = f'{sheath_device}/PFI4'
sheath_freq = 200
sheath_current_ds = 0.5
sheath_task = None



sheath_flow = 14
sheath_error_margin = 0.5

'''pid = PID(0.5, 0.1, 0.0, setpoint=sheath_flow)  # Kp, Ki, Kd
pid.sample_time = 0.5             
pid.output_limits = (0.0, 1.0) '''


sleep_time = 5  # seconds
meas_time = 10  # seconds per size

size_list_nm = [10, 15, 20, 25]

def cpc_read():
    cpc_con = None

    #https://www.manualslib.com/manual/1562828/Airmodus-A11.html?page=71#manual
    try:
        with serial.Serial(cpc_port, 115200, timeout=1) as ser_cpc:
            ser_cpc.write(b':MEAS:OPC\r') #Read OPC pulse duration (total dead time during averaging time), number of pulses,concentration, averaging time and DC compensation state
            cpc_con = ser_cpc.read_until(new_line).decode('utf-8').strip().replace(',', ' ').replace(':MEAS:OPC', '').strip().split(',')[2]
    except:
        print("CPC Error: Could not connect to CPC.")
        pass
    return cpc_con

def cunningham_correction(dp, T=293.15, P=101325):
    lambda_0 = 65e-9
    T0 = 273.15
    P0 = 101325
    lambda_air = lambda_0 * (T / T0) * (P0 / P)
    return 1 + (2 * lambda_air / dp) * (1.257 + 0.4 * np.exp(-1.1 * dp / (2 * lambda_air)))

def voltage_from_size(dp_nm, Q_sh_lpm=14.0, T_C=20.0, debug=False):
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

    Cc = cunningham_correction(dp, T_K)

    V = (3 * mu * Q_sh * ln_r * dp) / (2 * L * e * Cc)
    analog = V / (HV_MAX / 10.0)
    analog = np.clip(analog, 0.0, 10.0)

    if debug:   
        print(f"dp: {dp_nm} nm, Cc: {Cc:.3f}, HV: {V:.1f} V, analog: {analog:.3f} V")

    return analog

def set_daq_voltage(device_name, voltage):
    if device_name != "None":
        try:
            with nidaqmx.Task() as task:
                task.ao_channels.add_ao_voltage_chan(f"{device_name}/ao0")
                task.write(voltage)
        except Exception as e:
            print(f'DAQ Error: {e}')
            pass

    

def set_sheath_duty(duty_cycle):


    global sheath_task, sheath_current_ds, sheath_device, sheath_device_counter, sheath_device_output

    ds = float(np.clip(duty_cycle, 0, 1))

    if sheath_task is not None and abs(ds - sheath_current_ds) < 0.01:
        return
    
    if _fan_task is not None:
        try:
            _fan_task.stop()
        except Exception:
            pass
        _fan_task.close()
        _fan_task = None

        task = nidaqmx.Task()
        task.co_channels.add_co_pulse_chan_freq(
            f"{sheath_device}/{sheath_device_counter}",
            units=FrequencyUnits.HZ,
            idle_state=Level.LOW,
            initial_delay=0.0,
            freq=sheath_freq,
            duty_cycle=duty_cycle,
        )
        task.timing.cfg_implicit_timing(sample_mode=AcquisitionType.CONTINUOUS)

        task.export_signals.export_signal(
            Signal.CO_PULSE, sheath_device_output
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
            if line == 'OK':
                flow,temp,press= ser.read_until(b"\n").decode("ascii", errors="ignore").strip().split(',')
            return flow, temp, press
    except Exception as e:
        print(f"TSI flowmeter error: {e}")
        return None

def set_sheath_flow(sheath):
    pid = PID(0.5, 0.1, 0.0, setpoint=sheath)

    try:
        setpoint = float(sheath)
    except (TypeError, ValueError):
        print(f"Invalid sheath setpoint: {sheath}")
        return None

    pid.setpoint = setpoint

    flow = read_sheath_flow_mbed()
    if flow is None:
        return None

    duty = pid(flow)

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
            print(row)
            with open(get_log_filenames(), 'a', newline='') as logfile:
                writer = csv.writer(logfile)
                if not file_exists:
                    writer.writerow(row.keys())
                    file_exists = True
                writer.writerow(row.values())
            time.sleep(sleep_time)

if __name__ == "__main__":
    flow, temp, press = read_sheath_flow_mbed(flowmeter_baud=38400, flowmeter_port='/dev/ttyUSB0')
    