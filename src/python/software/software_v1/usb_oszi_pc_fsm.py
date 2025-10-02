import time
import queue
import numpy as np
from statemachine import StateMachine, State
from serial_interface import SerialInterface
from find_usb_device import find_my_device

datalength = 200_000
adc_resolution = 12             # ADC resolution in bit
vdiv_factor = (410+590)/410     # voltage divider in analog frontend

def calculate_voltage(raw_value, vref):
    return raw_value/4095 * vref * 1e-3 * vdiv_factor

# --- USB-Kommunikations-FSM ---
class usbOsziPC(StateMachine):
    init = State("Init", initial=True)
    idle = State("Idle")
    acquiring_start = State("Acquiring_Start")
    acquiring_stop = State("Acquiring_Stop")
    transmission = State("Transmission")
    wait_for_ack = State("Wait_For_Ack")

    app_started = init.to(idle)
    run = idle.to(acquiring_start)
    aquisition_done = acquiring_start.to(acquiring_stop)
    ready_for_data = acquiring_stop.to(transmission)
    data_transfer_complete = transmission.to(wait_for_ack)
    processing_complete = wait_for_ack.to(acquiring_start)
    stop_acquisition = wait_for_ack.to(idle)
    reset = (
        init.to(init)| idle.to(init) | acquiring_start.to(init) | acquiring_stop.to(init) |
        transmission.to(init) | wait_for_ack.to(init)
    )

def fsm_pc(command_queue, status_queue, stop_event):
    fsm = usbOsziPC()

    serial_port = find_my_device("Serielles USB")
    serial_baudrate = 115200
    serial_timeout = 5
    serial_interface = ''
    vref = 4095
 

    current_fsm_state = ""
    while not stop_event.is_set():
        # --- Kommando-Verarbeitung von der GUI ---
        try:
            command = command_queue.get(timeout=0.05)
            print(f"FSM received command: {command}")
            if command["type"] == "RESET":
                fsm.reset()
                print("FSM wurde resetet")
            elif command["type"] == "START_ACQUISITION" and fsm.current_state == fsm.idle:
                serial_interface.write("RUN_START")
                fsm.run() # Transition basierend auf GUI-Kommando

            elif command["type"] == "PROCESSING_COMPLETE" and fsm.current_state == fsm.wait_for_ack:
                if command.get("continue_acquisition", False):
                    serial_interface.write("CONTINUE")
                    fsm.processing_complete()
                else:
                    serial_interface.write("STOP")
                    status_queue.put({"type": "TRIGGER_TIMEOUT"})
                    fsm.stop_acquisition()

            elif command["type"] == "STOP_ACQUISITION" and fsm.current_state == fsm.wait_for_ack:
                serial_interface.write("STOP")
                status_queue.put({"type": "TRIGGER_TIMEOUT"})
                fsm.stop_acquisition()
            
            elif command["type"] == "SET_TRIGGER_EDGE" and fsm.current_state == fsm.idle:
                if fsm.current_state == fsm.idle:
                    edge_command = command["edge"] 
                    serial_interface.write(edge_command)
                    print(f"FSM: Sent trigger edge command to uC: {edge_command}") 
                    line = serial_interface.read_line()
                    if line:
                        print(f"FSM received message from uC: {line}")
                        status_queue.put({"type": "INFO", "message": f"{line}"})
                    else:
                        print("\nuC has not set trigger edge")
                else:
                    print(f"FSM: Cannot set trigger edge in current state: {fsm.current_state.id}")
                    status_queue.put({"type": "ERROR", "message": f"Cannot set trigger edge in {fsm.current_state.id} state."})

            elif command["type"] == "CONFIGURE_FREQ" and fsm.current_state == fsm.idle:
                if fsm.current_state == fsm.idle:
                    freq_command = command["type"] + " " + command["FREQ"]
                    serial_interface.write(freq_command)
                    print(f"FSM: Sent frequence configure command to uC: {freq_command}")
                    line = serial_interface.read_line()
                    if line:
                        print(f"FSM received message from uC: {line}")
                        status_queue.put({"type": "GET_FREQUENCY", "FREQUENCY": line})
                    else:
                        print("\nuC has not set frequency")
                else:
                    print(f"FSM: Cannot set frequency in current fsm state: {fsm.current_state.id}")
                    status_queue.put({"type": "ERROR", "message": f"Cannot set frequency in {fsm.current_state.id} state."})
            
            elif command["type"] == "CONFIGURE_REF_VOLT" and fsm.current_state == fsm.idle:
                ref_voltage_command = command["type"] + " " + command["REF"]
                serial_interface.write(ref_voltage_command)
                print(f"FSM: Sent reference voltage configure command to uC: {ref_voltage_command}")
                line = serial_interface.read_line()
                if line:
                    print(f"FSM received message from uC: {line}")
                    if line.startswith("REFERENCE_VOLTAGE: "):
                        vref_str = line.split(":")[1].strip()
                        vref = int(vref_str)
                    status_queue.put({"type": "GET_REF_VOLTAGE", "REF": line})

            status_queue.put({"type": "FSM_STATE", "state": fsm.current_state.id})
        except queue.Empty:
            pass # Kein Kommando von der GUI, weiter mit FSM-Logik

        


        # --- FSM Zustandsbasierte Logik für serielle Kommunikation ---
        current_fsm_state = fsm.current_state.id
        line_from_uc = ""
        if current_fsm_state in ["idle", "acquiring_start", "acquiring_stop", "wait_for_ack"]: # Nur in diesen Zuständen wird eine Zeile erwartet
            line_from_uc = serial_interface.read_line()
            if serial_interface.timeout_ocurred and current_fsm_state == "acquiring_stop":
                status_queue.put({"type": "TRIGGER_TIMEOUT"})


        if current_fsm_state  == "init":
            if serial_port:
                serial_interface = SerialInterface(serial_port, serial_baudrate, serial_timeout)
                if not serial_interface.is_connected:
                    serial_interface = SerialInterface(serial_port, serial_baudrate, serial_timeout)

                if not serial_interface.is_connected:
                     print("Failed to open serial port. Make sure that the uC is connected.")
                     status_queue.put({"type": "ERROR", "message": "Make sure the uC is connected and press reset again."})

                serial_interface.write("RESET")
                print(f"uC wurde resetet")

                serial_interface.write("APP")
                fsm.app_started() 
                status_queue.put({"type": "FSM_STATE", "state": fsm.current_state.id})
            else:
                time.sleep(1)
                serial_port = find_my_device("Serielles USB")
                status_queue.put({"type": "ERROR", "message": "Serial port not available."})


        elif current_fsm_state == "idle":
            if line_from_uc == "STATE:IDLE":
                print(f"[INFO] Got {line_from_uc}")
            

            elif line_from_uc:
                print(f"FSM (idle_start): Received unexpected line: {line_from_uc}")

        elif current_fsm_state == "acquiring_start":
            if line_from_uc == "STATE:ACQUISITION_START":
                print(f"[INFO] Got {line_from_uc}")
                fsm.aquisition_done()
                status_queue.put({"type": "FSM_STATE", "state": fsm.current_state.id})
            elif line_from_uc:
                print(f"FSM (acquiring_start): Received unexpected line: {line_from_uc}")

        elif current_fsm_state == "acquiring_stop":
            if line_from_uc == "STATE:ACQUISITION_DONE":
                print(f"FSM: uC signalled acquisition done.")
                status_queue.put({"type": "FSM_STATE", "state": fsm.current_state.id})
                status_queue.put({"type": "TRIGGERED"})
                #status_queue.put({"type": "DATA_READY_FOR_GUI", "message": "uC finished acquisition"})
                
                fsm.ready_for_data()
                status_queue.put({"type": "FSM_STATE", "state": fsm.current_state.id})

            elif line_from_uc:
                print(f"FSM (acquiring_start): Received unexpected line: {line_from_uc}")

        elif current_fsm_state == "transmission":
            
            #if line_from_uc == "STATE:TRANSMISSION":
                #print(f"[INFO] Got {line_from_uc}")
                # --- Datenempfang ---
                serial_interface.write("READY_FOR_DATA")    # wird erst hier gemacht, weil die Zeit, die für den Transitionsübergang benötigt wird zu hoch ist, so dass der Databuffer überflutet wird
                # State des uC wird hier nicht überprüft, da zeitkritisch
                time1 = time.perf_counter()
                received_data_bytes = serial_interface.read(datalength) # Lese Rohdaten
                time2 = time.perf_counter()
                overflow_byte = serial_interface.read(1)
        
                if overflow_byte == b'\xff':
                    status_queue.put({"type": "OVERFLOW"})
                    print("\nOverflow happend")
                else:
                    status_queue.put({"type": "NO_OVERFLOW"})
                print('Transmission time: {0}'.format(time2 - time1))
                status_queue.put({"type": "Transmission_Time", "time": (time2-time1)})
                print('Received Bytes: {0}'.format(len(received_data_bytes)))

                if len(received_data_bytes) != datalength:
                    print(f"WARNING: Expected {datalength} bytes but received {len(received_data_bytes)} bytes. Data might be incomplete.")

                # Konvertiere Byte-String in eine Liste von 2-Byte-Integern
                time1 = time.perf_counter()

                data_arr = np.frombuffer(received_data_bytes, dtype='<i2')
                voltage_res = calculate_voltage(data_arr, vref)

                time2 = time.perf_counter()
                status_queue.put({"type": "Conversion_Time", "time": (time2-time1)})
                print('Conversion time: {0}'.format(time2 - time1))

                status_queue.put({"type": "RAW_DATA", "data": voltage_res})
                serial_interface.write("RECEIVED") 
                fsm.data_transfer_complete() 
                status_queue.put({"type": "FSM_STATE", "state": fsm.current_state.id})
                print(f"[INFO] Current state: {fsm.current_state}")
            

        if fsm.current_state.id == "wait_for_ack":
            line_from_uc = serial_interface.read_line()
            if line_from_uc == "STATE:WAIT_FOR_ACK":
                print(f"[INFO] Got {line_from_uc}")
            
                    
        time.sleep(0.01)

    print("FSM thread stopping.")
    if serial_interface:
        serial_interface.close()