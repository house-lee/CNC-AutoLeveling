import sys
import getopt
import time
import serial

import RPi.GPIO as io
io.setmode(io.BCM)
probe_pin = 24
PROBE_TOUCHED = 0

# maximum z travel distance of the spindle/bits for each probe
# this is very important in terms of safety. w/o a proper value, we may risk of damaging the machine.
max_probe_tavel=5 #mm 

current_x = 0.0
current_y = 0.0
current_z = 0.0
default_mv_speed = 1800 #mm/min

start_x = 0.0
start_y = 0.0
start_set = False
end_x = 0.0
end_y = 0.0
end_set = False

rpf_savepath="~/cnc.rpf"

def execute_gcode(ser:serial.Serial, command:str, skip_log=False) -> str:
    if not skip_log:
        print("executing [{cmd}]".format(cmd=command))
    ser.write((command + "\r\n").encode())
    resp = ser.read_until()
    return resp.decode().strip(" \r\n")

def get_current_position(ser:serial.Serial) -> tuple[float, float, float]:
    resp = execute_gcode(ser, "M114")
    idx = resp.find("Count")
    max_try = 5
    attempt = 0
    while idx == -1 and attempt < max_try:
        resp = ser.read_until().decode().strip(" \r\n")
        idx = resp.find("Count")
        attempt += 1
        # print(resp)
    
    resp = resp[:idx].strip().split()
    x = 0.0
    y = 0.0
    z = 0.0
    for item in resp:
        v = item.split(":")
        if len(v) == 2:
            if v[0] == 'X':
                x = float(v[1])
            elif v[0] == 'Y':
                y = float(v[1])
            elif v[0] == 'Z':
                z = float(v[1])
    return (x,y,z)
    

def list_ports():
    """
    list all serial ports
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')
    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    print("serial ports:")
    for res in result:
        print(res)

def move(ser:serial.Serial, argv):
    global current_x, current_y, current_z, default_mv_speed
    dx = 0
    dy = 0
    dz = 0
    speed = default_mv_speed
    for arg in argv:
        delta = arg.split(":")
        if len(delta) == 2:
            if delta[0].lower() == 'x':
                dx=float(delta[1])
            elif delta[0].lower() == 'y':
                dy=float(delta[1])
            elif delta[0].lower() == 'z':
                dz=float(delta[1])
            elif delta[0].lower() == 's':
                speed=int(delta[1])
    end_x = current_x + dx
    end_y = current_y + dy
    end_z = current_z + dz
    print("moving from (X{sx:.3f},Y{sy:.3f},Z{sz:.3f}) to (X{ex:.3f},Y{ey:.3f},Z{ez:.3f}) at speed:{speed}mm/min".format(
        sx=current_x,sy=current_y,sz=current_z,
        ex=end_x,ey=end_y,ez=end_z,
        speed=speed
        ))
    execute_gcode(ser, "G1 X{x:.3f} Y{y:.3f} Z{z:.3f} F{s}".format(x=end_x, y=end_y, z=end_z, s=speed))
    (current_x, current_y, current_z) = get_current_position(ser)
    print("current posistion:(X:{x:.3f},Y:{y:.3f},Z:{z:.3f})".format(x=current_x, y=current_y, z=current_z))

def set_start_pt():
    global start_x, start_y
    global current_x, current_y
    global start_set
    print("setting start coordinate to (X{x:.3f}, Y{y:.3f})".format(x=current_x, y=current_y))
    (start_x, start_y) = (current_x, current_y)
    start_set = True

def set_end_pt():
    global end_x, end_y
    global current_x, current_y
    global end_set
    print("setting end coordinate to (X{x:.3f}, Y{y:.3f})".format(x=current_x, y=current_y))
    (end_x, end_y) = (current_x, current_y)
    end_set = True

def probe_z(ser:serial.Serial, start_z:float, normal_speed:int)->float:
    """
    probe_z attempts to probe the z value for the current (x,y) position. 
    it lower the z axis at the given step until it touches the surface
    then the z value of the surface will be returned
    """
    global max_probe_tavel
    # the travel distance of the spindle/probe for each step
    # note: 0.1mm is just for experiment, in real life it should be smaller to have a finer granularity
    step_z = 0.1 #mm
    max=max_probe_tavel/step_z #maximum number of steps
    speed=5*60 #5mm/s
    z = start_z
    for i in range(0,int(max)):
        z -= step_z
        # using G1 command with a much slower speed to gradually lower the z-axis until touching the surface
        execute_gcode(ser, "G1 Z{z:.3f} F{s}".format(z=z, s=speed), skip_log=True)
        # wait a bit to 
        #  - wait for gcode to execute, and
        #  - make reading the pin input more stable
        # note: the value 0.05 is calculated as travel_step(0.1mm)/speed(5mm/s) + short_delay(0.03s)
        # should probably refactor hardcoded value to be a variable with the above equation applied
        # note: I also tried executing a M400 gcode here, but it will cause a hard stop of the z-axis of each step,
        # which makes it like putting the machine into a "virbation" mode when probing, I'm not sure whether this kind
        # of vibration will cause any harm to the machine step motor
        time.sleep(0.05) 
        touched = io.input(probe_pin)
        if touched == PROBE_TOUCHED:
            break
    # use another G1 command with a faster speed to return Z to original position 
    execute_gcode(ser, "G1 Z{z:.3f} F{s}".format(z=start_z, s=normal_speed), skip_log=True)
    execute_gcode(ser, "M400")
    return z

def current_posistion():
    global current_x, current_y, current_z
    print("current posistion:(X:{x:.3f},Y:{y:.3f},Z:{z:.3f})".format(x=current_x, y=current_y, z=current_z))

def probe(ser:serial.Serial, step:float):
    global start_x, start_y, start_set
    global end_x, end_y, end_set
    global current_z, default_mv_speed
    global rpf_savepath
    if not start_set:
        print("starting point has not been set yet")
        return
    if not end_set:
        print("ending point has not been set yet")
        return
    try:
        f = open(rpf_savepath, "w")
    except OSError:
        print("failed to open file for writing. file:" + rpf_savepath)
        sys.exit(-1)
    execute_gcode(ser, "G0 X{x:.3f} Y{y:.3f}".format(x=start_x, y=start_y))
    execute_gcode(ser, "M400")
    print("probing from (X{sx:.3f}, Y{sy:.3f}) to (X{ex:.3f}, Y{ey:.3f}) with step {step}mm".format(
        sx=start_x,
        sy=start_y,
        ex=end_x,
        ey=end_y,
        step=step
    ))
    step_x = step if end_x > start_x else -step
    step_y = step if end_y > start_y else -step
    x = start_x
    y = start_y
    while (y <= end_y and end_y > start_y) or (y >= end_y and end_y < start_y):
        while (x <= end_x and end_x > start_x) or (x >= end_x and end_x < start_x):
            execute_gcode(ser, "G1 X{x:.3f}".format(x=x))
            z=probe_z(ser, current_z, default_mv_speed)
            print("(X{x:.3f}, Y{y:.3f}, Z{z:.3f}) ".format(x=x,y=y,z=z))
            f.write("{x:.3f}, {y:.3f}, {z:.3f}\n".format(x=x,y=y,z=z))
            x += step_x
            execute_gcode(ser, "M400") #finish a row, wait for all gcode execution finish before proceeding
        print("y=",y)
        x = start_x
        y += step_y
        execute_gcode(ser, "G1 X{x:.3f} Y{y:.3f}".format(x=x, y=y))
        execute_gcode(ser, "M400")
    f.close()



def main(argv):
    global current_x, current_y, current_z
    global probe_pin
    global rpf_savepath
    io.setup(probe_pin, io.IN, pull_up_down=io.PUD_UP)
    argv = argv[1:]
    opts, _ = getopt.getopt(argv, "p:s:o:l",
                            ["port=",
                             "step=",
                             "output=",
                             "list_ports"])
    serialPort = ""
    step = 10 #default to 10mm
    for opt, arg in opts:
        if opt in ['-l', '--list_ports']:
            list_ports()
            return
        elif opt in ['-p', '--port']:
            print("port:", arg)
            serialPort = arg
        elif opt in ['-s', '--step']:
            print("probe step (mm):", arg)
            step = arg
        elif opt in ['-o', '--output']:
            print("the generated raw probe file (rpf) will be saved to:", arg)
            rpf_savepath = arg
        else:
            print("invalid option:"+opt)
    try:
        ser = serial.Serial(
            port=serialPort,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=2
        )
    except (OSError, serial.SerialException):
        print("invalid serial port:" + serialPort)
    if not ser.isOpen():
        print("could not open serial port:" + serialPort)
        return
    ser.flushInput()
    ser.flushOutput()

    resp = execute_gcode(ser, "G21 G90")
    print("result:[{resp}]".format(resp=resp))
    (current_x,current_y,current_z) = get_current_position(ser)
    print("current posistion:(X:{x:.3f},Y:{y:.3f},Z:{z:.3f})".format(x=current_x, y=current_y, z=current_z))
    # print(execute_gcode(ser, "G1 Z100 F1800"))
    # for i in range(1,20):
    #     z -= 0.1
    #     print(execute_gcode(ser, "G0 Z{z:.3f}".format(z=z)))
        # print(execute_gcode(ser, "M400"))
    while True:
        print(">", end="", flush=True)
        line = sys.stdin.readline()
        argv = line.split()
        if len(argv) == 0:
            continue
        if argv[0].lower() == "mv":
            move(ser, argv[1:])
        elif argv[0].lower() == "set_start":
            set_start_pt()
        elif argv[0].lower() == "set_end":
            set_end_pt()
        elif argv[0].lower() == "probe":
            probe(ser, step)
        elif argv[0].lower() == "pos":
            current_posistion()


if __name__ == "__main__":
    main(sys.argv)
