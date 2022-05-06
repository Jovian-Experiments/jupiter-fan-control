#!/usr/bin/python -u
import signal
import time
import math
import yaml
import os
from PID import PID

# quadratic function RPM = AT^2 + BT + X
class Quadratic(object):
    def __init__(self, A, B, C, T_threshold):
        self.A = A
        self.B = B
        self.C = C
        self.T_threshold = T_threshold
        self.output = 0
    
    def update(self, temp_input, _):
        if temp_input < self.T_threshold:
            return 0
        self.output = int(self.A * math.pow(temp_input, 2) + self.B * temp_input + self.C)
        return max(0, self.output)

class Hybrid(object):
    def __init__(self, slope, A_setpoint, B_setpoint, T_setpoint):
        self.slope = slope
        self.A_setpoint = A_setpoint
        self.B_setpoint = B_setpoint
        self.T_setpoint = T_setpoint
        self.output = 0

    '''
    input is the linear setpoint curve (power in, output is RPM that in steady state will cool unit to a constant temperature) and the temp
        A_setpoint * Power + B_setpoint = RPM_center

    get_setpoint(power) -> RPM

    get_curve(rpm) -> A_output, B_output

    '''

    def get_setpoint(self, power_input):
        rpm_setpoint = self.A_setpoint * power_input + self.B_setpoint
        return rpm_setpoint

    def get_curve(self, rpm_setpoint):
        # we are finding the equation for the line that passes through (rpm_setpoint, T_setpoint) at slope self.slope
        A_output = self.slope
        B_output = rpm_setpoint - A_output * self.T_setpoint
        return A_output, B_output

    def update(self, temp_input, power_input):
        A, B = self.get_curve(self.get_setpoint(power_input))
        self.output = A * temp_input + B
        return self.output



# fan object controls all jupiter hwmon parameters
class Fan(object):
    def __init__(self, fan_path, config, debug = False) -> None:
        self.debug = debug
        self.min_speed = config["fan_min_speed"]
        self.threshold_speed = config["fan_threshold_speed"]
        self.max_speed = config["fan_max_speed"]
        self.gain = config["fan_gain"]
        self.fan_path = fan_path
        self.fc_speed = 0
        self.measured_speed = 0
        self.ec_ramp_rate = config["ec_ramp_rate"]
        # self.ramp_up_rate = 400
        # self.ramp_down_rate = -50
        self.take_control_from_ec()
        self.set_speed(0)

    def take_control_from_ec(self):
        with open(self.fan_path + "gain", 'w') as f:
            f.write(str(self.gain))
        with open(self.fan_path + "ramp_rate", 'w') as f:
            f.write(str(self.ec_ramp_rate))
        with open(self.fan_path + "recalculate", 'w') as f:
            f.write(str(1))


    def get_speed(self):
        with open(self.fan_path + "fan1_input", 'r') as f:
            self.measured_speed = int(f.read().strip())
        return self.measured_speed

    # def set_speed(self, speed):
    #     if speed > self.max_speed:
    #         speed = self.max_speed
    #     if speed < self.threshold_speed:
    #         speed = self.min_speed

    #     error = speed - self.fc_speed
    #     if error >= 0:
    #         if error <= self.ramp_up_rate:
    #             self.fc_speed = speed
    #         else:
    #             self.fc_speed += self.ramp_up_rate
    #     else:
    #         if error >= self.ramp_down_rate:
    #             self.fc_speed = speed
    #         else:
    #             self.fc_speed += self.ramp_down_rate

    #     with open(self.fan_path + "fan1_target", 'w') as f:
    #         f.write(str(int(self.fc_speed)))

    def set_speed(self, speed):
        if speed > self.max_speed:
            speed = self.max_speed
        if speed < self.threshold_speed:
            speed = self.min_speed
        self.fc_speed = speed
       
        with open(self.fan_path + "fan1_target", 'w') as f:
            f.write(str(int(self.fc_speed)))


    def return_to_ec_control(self):
        with open(self.fan_path + "gain", 'w') as f:
            f.write(str(10))
        with open(self.fan_path + "ramp_rate", 'w') as f:
            f.write(str(20))
        with open(self.fan_path + "recalculate", 'w') as f:
            f.write(str(0))

# devices are sources of heat - CPU, GPU, etc.
class Device(object):
    def __init__(self, base_path, config, fan_max_speed, n_sample_avg, debug = False) -> None:
        self.debug = debug
        self.fan_max_speed = fan_max_speed
        self.n_sample_avg = n_sample_avg
        self.nice_name = config["nice_name"]
        self.file_path = get_full_path(base_path, config["hwmon_name"]) + config["file"]
        self.max_temp = config["max_temp"]
        self.temp_deadzone = config["temp_deadzone"]
        self.temp = 0
        self.control_temp = 0 # deadzone temp
        self.control_output = 0 # controller output if > 0, max fan speed if max temp reached
        self.buffer_full = False
        self.control_temps = []
        self.avg_control_temp = 0

        # instantiate controller depending on type
        self.type = config["type"]
        if self.type == "pid":
            # testing out scaling PID coefficiencts down with bandwidth, so we can tune with less dependance on bandwidth
            self.controller = PID(float(config["Kp"]), float(config["Ki"]), float(config["Kd"]))  
            self.controller.SetPoint = config["T_setpoint"]
            self.controller.setWindup(config["windup_limit"]) # windup limits the I term of the output
        elif self.type ==  "quadratic":
            self.controller = Quadratic(float(config["A"]), float(config["B"]), float(config["C"]), float(config["T_threshold"]))
        elif self.type == "hybrid":
            self.controller = Hybrid(float(config["slope"]), float(config["A_setpoint"]), float(config["B_setpoint"]), int(config["T_setpoint"]))
        else:
            print("error loading device controller \n")
            exit(1)

    # updates temperatures
    def get_temp(self) -> None:
        with open(self.file_path, 'r') as f:
            self.temp = int(f.read().strip()) / 1000
        # only update the control temp if it's outside temp_deadzone
        if math.fabs(self.temp - self.control_temp) >= self.temp_deadzone:
            self.control_temp = self.temp
        return self.control_temp

    def get_avg_temp(self):
        self.control_temps.append(self.get_temp())
        if self.buffer_full:
            self.control_temps.pop(0)
        elif len(self.control_temps) >= self.n_sample_avg:
            self.buffer_full = True
        self.avg_control_temp = math.fsum(self.control_temps) / len(self.control_temps)
        return self.avg_control_temp


    def get_output(self, temp_input, power_input):
        self.controller.update(temp_input, power_input)
        self.control_output = max(self.controller.output, 0)
        if(temp_input > self.max_temp):
            self.control_output = self.fan_max_speed
        return self.control_output

# testing variable PID setpoints
class Sensor(object):
    def __init__(self, base_path, config, debug = False) -> None:
        self.debug = debug
        self.nice_name = config["nice_name"]
        self.file_path = get_full_path(base_path, config["hwmon_name"]) + config["file"]
        self.n_sample_avg = config["n_sample_avg"]
        self.value = 0
        self.avg_value = 0
        self.buffer_full = False
        self.values = []

    def get_value(self):
        with open(self.file_path, 'r') as f:
            self.value = int(f.read().strip()) / 1000000
        return self.value
    
    def get_avg_value(self) -> float:
        self.values.append(self.get_value())
        if self.buffer_full:
            self.values.pop(0)
        elif len(self.values) >= self.n_sample_avg:
            self.buffer_full = True
        self.avg_value = math.fsum(self.values) / len(self.values)
        return self.avg_value



# helper function to find correct hwmon* path for a given device name
def get_full_path(base_path, name):
    for directory in os.listdir(base_path):
        full_path = base_path + directory + '/'
        test_name = open(full_path + "name").read().strip()
        if test_name == name:
            return full_path
    print("failed to find device {}".format(name))

# main FanController class
class FanController(object):
    def __init__(self, debug, config_file):
        self.debug = debug

        # read in config yaml file
        if debug: print("reading config file")
        with open(config_file, "r") as f:
            try:
                self.config = yaml.safe_load(f)
            except yaml.YAMLError as exc:
                print("error loading config file \n", exc)
                exit(1)

        # store global parameters
        self.base_hwmon_path = self.config["base_hwmon_path"]
        self.loop_interval = self.config["loop_interval"]
        self.control_to_sense_ratio = self.config["control_to_sense_ratio"]

        # initialize fan
        fan_path = get_full_path(self.base_hwmon_path, self.config["fan_hwmon_name"])
        self.fan = Fan(fan_path, self.config, self.debug) 

        # initialize list of devices
        self.devices = [ Device(self.base_hwmon_path, device_config, self.fan.max_speed, self.control_to_sense_ratio, self.debug) for device_config in self.config["devices"] ]

        # initialize APU power sensor
        self.power_sensor = Sensor(self.base_hwmon_path, self.config["sensors"][0], self.debug)

        # exit handler
        signal.signal(signal.SIGTERM, self.on_exit)

    # pretty print all device values, temp source, and output
    def print_single(self, source_name):
        for device in self.devices:
                print("{}: {:.1f}/{:.0f}  ".format(device.nice_name, device.temp, device.control_output), end = '')
                #print("{}: {}  ".format(device.nice_name, device.temp), end = '')
        print("{}: {:.1f}/{:.1f}  ".format(self.power_sensor.nice_name, self.power_sensor.value, self.power_sensor.avg_value), end = '')

        print("Fan[{}]: {}/{}".format(source_name, int(self.fan.fc_speed), self.fan.measured_speed))

    def read_sensors(self):
        start_time = time.time()

        self.power_sensor.get_avg_value()

        for device in self.devices:
            device.get_temp()

        sleep_time = self.loop_interval - (time.time() - start_time)
        if sleep_time > 0:
            time.sleep(sleep_time)

    # automatic control loop
    def loop_control(self):
        while True:
            fan_error = abs(self.fan.fc_speed - self.fan.get_speed())
            if fan_error > 500:
                self.fan.take_control_from_ec()

            for _ in range(self.control_to_sense_ratio):
                self.read_sensors()

            # get outputs
            for device in self.devices:
                device.get_output(device.control_temp, self.power_sensor.avg_value)

            max_output = max(device.control_output for device in self.devices)
            self.fan.set_speed(max_output)

            # find source name for the max control output
            source_name = next(device for device in self.devices if device.control_output == max_output).nice_name
            # print all values
            self.print_single(source_name)

            # # sleep until interval is complete
            # sleep_time = self.loop_interval - (time.time() - start_time)
            # if sleep_time > 0:
            #     time.sleep(sleep_time)
            # else: 
            #     if self.debug:
            #         print("over-ran specified interval, skipping sleep")
    
    def on_exit(self, signum, frame):
        self.fan.return_to_ec_control()
        print("returning fan to EC control loop")
        exit()

# main loop
if __name__ == '__main__':
    print('jupiter-fan-control starting up ...')

    # specify config file path
    # config_file_path = os.getcwd() + "/config.yaml"
    config_file_path = "/usr/share/jupiter-fan-control/jupiter-fan-control-config.yaml"

    # initialize controller
    controller = FanController(debug = False, config_file = config_file_path)

    # start main loop
    controller.loop_control()
    
    print('jupiter-fan-control startup complete')
