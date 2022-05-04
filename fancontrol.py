#!/usr/bin/python -u
import signal
import time
import math
import yaml
import os
from PID import PID

# quadratic function RPM = AT^2 + BT + X
class Quadratic(object):
    def __init__(self, A, B, C, T_threshold, power):
        self.A = A
        self.B = B
        self.C = C
        self.T_threshold = T_threshold
        self.power_coefficient = power
        self.output = 0
    
    def update(self, temp_input):
        if temp_input < self.T_threshold:
            return 0
        self.output = int(self.A * math.pow(temp_input, 2) + self.B * temp_input + self.C)
        return max(0, self.output)

    def update(self, temp_input, power_input):
        if temp_input < self.T_threshold:
            return 0
        self.output = int(self.A * math.pow(temp_input, 2) + self.B * temp_input + self.C + power_input * self.power_coefficient)
        return max(0, self.output)


# exponential function RPM = A * exp(B * T) if T > T_threshold
class Exponential(object):
    def __init__(self, A, B, T_threshold):
        self.A = A
        self.B = B
        self.T_threshold = T_threshold
        self.output = 0
    
    def update(self, temp_input):
        if temp_input < self.T_threshold:
            return 0
        self.output = int(self.A * math.exp(self.B * temp_input))
        return max(0, self.output)

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



# fan object controls all jupiter hwmon parameters
class Fan(object):
    def __init__(self, fan_path, fan_min_speed, fan_threshold_speed, fan_max_speed, fan_gain, debug = False) -> None:
        self.debug = debug
        self.min_speed = fan_min_speed
        self.threshold_speed = fan_threshold_speed
        self.max_speed = fan_max_speed
        self.gain = fan_gain
        self.fan_path = fan_path
        self.fc_speed = 0
        self.measured_speed = 0
        self.ec_ramp_rate = 10
        self.ramp_up_rate = 400
        self.ramp_down_rate = -50
        self.take_control_from_ec()
        self.set_speed(0)

        # with open(self.fan_path + "ramp_rate", 'w') as f:
        #     f.write(str(1))

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

    def set_speed(self, speed):
        if speed > self.max_speed:
            speed = self.max_speed
        if speed < self.threshold_speed:
            speed = self.min_speed

        error = speed - self.fc_speed
        if error >= 0:
            if error <= self.ramp_up_rate:
                self.fc_speed = speed
            else:
                self.fc_speed += self.ramp_up_rate
        else:
            if error >= self.ramp_down_rate:
                self.fc_speed = speed
            else:
                self.fc_speed += self.ramp_down_rate

        with open(self.fan_path + "fan1_target", 'w') as f:
            f.write(str(int(self.fc_speed)))


    def return_to_ec_control(self):
        with open(self.fan_path + "gain", 'w') as f:
            f.write(str(10))
        with open(self.fan_path + "recalculate", 'w') as f:
            f.write(str(0))

# devices are sources of heat - CPU, GPU, etc.
class Device(object):
    def __init__(self, base_path, config, debug = False) -> None:
        self.debug = debug
        self.nice_name = config["nice_name"]
        self.file_path = get_full_path(base_path, config["hwmon_name"]) + config["file"]
        self.max_temp = config["max_temp"]
        self.temp_deadzone = config["temp_deadzone"]
        self.temp = 0
        self.control_temp = 0 
        self.type = config["type"]
        if self.type == "pid":
            self.bandwidth = config["bandwidth"]
            # testing out scaling PID coefficiencts down with bandwidth, so we can tune with less dependance on bandwidth
            self.controller = PID(float(config["Kp"] / self.bandwidth), float(config["Ki"]), float(config["Kd"]))  
            self.controller.SetPoint = self.max_temp - self.bandwidth
            self.controller.setWindup(config["windup"]) # windup limits the I term of the output
        elif self.type ==  "exponential":
            self.controller = Exponential(float(config["A"]), float(config["B"]), float(config["T_threshold"]))
        elif self.type ==  "quadratic":
            self.controller = Quadratic(float(config["A"]), float(config["B"]), float(config["C"]), float(config["T_threshold"]), 120)   #, float(config["P_coeff"]))
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

    # returns PID control output, or MAX
    def get_output(self, temp_input):
        output = self.controller.update(temp_input)
        if(temp_input > self.max_temp):
            return "max"
        else:
            return max(output, 0)

    def get_output(self, temp_input, power_input):
        output = self.controller.update(temp_input, power_input)
        if(temp_input > self.max_temp):
            return "max"
        else:
            return max(output, 0)

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

        # initialize list of devices
        self.devices = [ Device(self.base_hwmon_path, device_config, self.debug) for device_config in self.config["devices"] ]

        # initialize list of sensors
        self.sensors = [ Sensor(self.base_hwmon_path, sensor_config, self.debug) for sensor_config in self.config["sensors"] ]

        # initialize fan
        fan_path = get_full_path(self.base_hwmon_path, self.config["fan_hwmon_name"])
        self.fan = Fan(fan_path, self.config["fan_min_speed"], self.config["fan_threshold_speed"], self.config["fan_max_speed"], self.config["fan_gain"], self.debug) 

        # exit handler
        signal.signal(signal.SIGTERM, self.on_exit)

    # pretty print all device values, temp source, and output
    def print_single(self, source_name):
        for device in self.devices:
                print("{}: {:.1f}/{:.0f}  ".format(device.nice_name, device.temp, device.controller.output), end = '')
                #print("{}: {}  ".format(device.nice_name, device.temp), end = '')
        for sensor in self.sensors:
            print("{}: {:.1f}/{:.1f}  ".format(sensor.nice_name, sensor.value, sensor.avg_value), end = '')

        print("Fan[{}]: {}/{}".format(source_name, int(self.fan.fc_speed), self.fan.measured_speed))

    # automatic control loop
    def loop_control(self):
        while True:
            start_time = time.time()
            outputs = []
            names = []
            power = 0

            # names = ( [device.nice_name for device in self.devices] ) if want to move to tuples for perf

            fan_error = abs(self.fan.fc_speed - self.fan.get_speed())
            if fan_error > 500:
                self.fan.take_control_from_ec()

            for sensor in self.sensors:
                power = sensor.get_avg_value()

            # check temperatures
            for device in self.devices:
                device.get_temp()
                outputs.append(device.get_output(device.control_temp, power))
                names.append(device.nice_name)

            if "max" in outputs: # check if any devices were overtemp
                source_index = outputs.index("max")
                # set fan speed to max_speed
                self.fan.set_speed(self.fan.max_speed)
            else:
                # returns the index of the _highest output_, which is used to command the fan
                source_index = max(range(len(outputs)), key=outputs.__getitem__) 
                # set fan speed to output
                self.fan.set_speed(outputs[source_index])

            # record the name of the device that gave the highest output
            source_name = names[source_index]

            # print all values
            self.print_single(source_name)

            # sleep until interval is complete
            sleep_time = self.loop_interval - (time.time() - start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
            else: 
                if self.debug:
                    print("over-ran specified interval, skipping sleep")
    
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
