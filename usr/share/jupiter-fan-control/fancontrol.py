#!/usr/bin/python -u
"""jupiter-fan-controller"""
import signal
import os
import sys
from pathlib import Path
from collections import deque
import time
import math
import yaml
import csv
import argparse
from PID import PID


# quadratic function RPM = AT^2 + BT + C
class Quadratic:
    """quadratic function controller"""

    def __init__(self, A, B, C, T_threshold=0) -> None:
        """constructor"""
        self.A = A
        self.B = B
        self.C = C
        self.T_threshold = T_threshold
        self.output = 0

    def update(self, temp_input, _) -> int:
        """update output"""
        if temp_input < self.T_threshold:
            self.output = 0
        else:
            self.output = int(
                self.A * math.pow(temp_input, 2) + self.B * temp_input + self.C
            )
        return self.output


class FeedForward:
    """RPM predicted by APU power is fed forward + PID output stage"""

    def __init__(self, Kp, Ki, Kd, windup, winddown, a_ff, b_ff, temp_setpoint) -> None:
        """constructor"""
        self.a_ff = a_ff
        self.b_ff = b_ff
        self.temp_setpoint = temp_setpoint
        self.pid = PID(Kp, Ki, Kd)
        self.pid.SetPoint = temp_setpoint
        self.pid.setWindup(windup)
        self.pid.setWinddown(winddown)
        self.output = 0

    def print_ff_state(self, ff_output, pid_output) -> str:
        """prints state variables of FF and PID, helpful for debug"""
        print(
            f"FeedForward Controller - FF:{ff_output:.0f}    PID: {-1 * self.pid.PTerm:.0f}  {-1 * self.pid.Ki * self.pid.ITerm:.0f}  {-1 * self.pid.Kd * self.pid.DTerm:.0f} = {pid_output:.0f}"
        )

    def get_ff_setpoint(self, power_input) -> int:
        """returns the feed forward portion of the controller output"""
        rpm_setpoint = int(self.a_ff * power_input + self.b_ff)
        return rpm_setpoint

    def update(self, temp_input, power_input) -> int:
        """run controller to update output"""
        pid_output = self.pid.update(temp_input)
        ff_output = self.get_ff_setpoint(power_input)
        self.output = int(pid_output + ff_output)
        # self.print_ff_state(ff_output, pid_output)
        return self.output


class FeedForwardMin:
    """FF with an additional min curve"""

    def __init__(
        self, Kp, Ki, Kd, windup, winddown, a_ff, b_ff, temp_setpoint, a_min, b_min
    ) -> None:
        """constructor"""
        self.a_ff = a_ff
        self.b_ff = b_ff
        self.a_min = a_min
        self.b_min = b_min
        self.temp_setpoint = temp_setpoint
        self.pid = PID(Kp, Ki, Kd)
        self.pid.SetPoint = temp_setpoint
        self.pid.setWindup(windup)
        self.pid.setWinddown(winddown)
        self.output = 0

    def print_ff_state(self, ff_output, pid_output, min_setpoint) -> str:
        """prints state variables of FF and PID, helpful for debug"""
        print(
            f"FeedForward Controller - Min:{min_setpoint}    FF:{ff_output:.0f}    PID:{-1 * self.pid.PTerm:.0f}/{-1 * self.pid.Ki * self.pid.ITerm:.0f}/{-1 * self.pid.Kd * self.pid.DTerm:.0f} = {ff_output + pid_output:.0f}"
        )

    def get_ff_setpoint(self, power_input) -> int:
        """returns the feed forward portion of the controller output"""
        rpm_setpoint = int(self.a_ff * power_input + self.b_ff)
        return rpm_setpoint

    def get_min_setpoint(self, temp_input) -> int:
        """returns a minimum rpm speed for the given temperature"""
        rpm_setpoint = int(self.a_min * temp_input + self.b_min)
        return rpm_setpoint

    def update(self, temp_input, power_input) -> int:
        """run controller to update output"""
        pid_output = int(self.pid.update(temp_input))
        ff_output = self.get_ff_setpoint(power_input)
        min_setpoint = self.get_min_setpoint(temp_input)
        self.output = max(min_setpoint, (pid_output + ff_output))
        # self.print_ff_state(ff_output, pid_output, min_setpoint)
        return self.output


class FeedForwardQuad:
    """FF with an additional min curve"""

    def __init__(self, a_quad, b_quad, c_quad, a_ff, b_ff) -> None:
        """constructor"""
        self.a_ff = a_ff
        self.b_ff = b_ff
        # self.temp_setpoint = temp_setpoint
        self.ff_deadzone = 300
        self.ff_last_setpoint = 0
        self.quad = Quadratic(a_quad, b_quad, c_quad)
        self.output = 0

    def print_ff_state(self, ff_output, quad_output):
        """prints state variables of FF and PID, helpful for debug"""
        print(f"FeedForward Controller - Quad:{quad_output}    FF:{ff_output:.0f}")

    def get_ff_setpoint(self, power_input) -> int:
        """returns the feed forward portion of the controller output"""
        rpm_setpoint = int(self.a_ff * power_input + self.b_ff)
        if abs(rpm_setpoint - self.ff_last_setpoint) > self.ff_deadzone:
            self.ff_last_setpoint = rpm_setpoint
            return rpm_setpoint
        return self.ff_last_setpoint

    def update(self, temp_input, power_input) -> int:
        """run controller to update output"""
        quad_output = int(self.quad.update(temp_input, None))
        ff_output = self.get_ff_setpoint(power_input)
        # min_setpoint = self.get_min_setpoint(temp_input)
        self.output = quad_output + ff_output
        # self.print_ff_state(ff_output, quad_output)
        return self.output


class DmiId:
    def __init__(self) -> None:
        self.id = Path("/sys/class/dmi/id")
        self.bios_version = self.read("bios_version")
        self.board_name = self.read("board_name")

    def read(self, identifier):
        with open(self.id / identifier, "r", encoding="utf-8") as file:
            return file.read().strip()


class Fan:
    """fan object controls all jupiter hwmon parameters"""

    def __init__(self, fan_path, config, dmi) -> None:
        """constructor"""
        self.fan_path = fan_path
        self.charge_state_path = config["charge_state_path"]
        self.min_speed = config["fan_min_speed"]
        self.threshold_speed = config["fan_threshold_speed"]
        self.max_speed = config["fan_max_speed"]
        self.min_time_on = config["fan_min_time_on"]
        self.gain = config["fan_gain"]
        self.ec_ramp_rate = config["ec_ramp_rate"]
        self.fc_speed = 0
        self.measured_speed = 0
        self.time_on = 0
        self.cold_off = True
        self.charge_state = False
        self.charge_min_speed = self.threshold_speed
        self.has_std_bios = self.bios_compatibility_check(dmi)
        self.take_control_from_ec()
        self.set_speed(2000)

    @staticmethod
    def bios_compatibility_check(dmi: DmiId) -> bool:
        """returns True for bios versions >= 106, false for earlier versions"""
        model = dmi.bios_version[0:3]
        version = int(dmi.bios_version[3:7])
        # print("model: ", model, " version: ", version)

        if model.find("F7A") != -1:
            if version >= 106:
                return True
            else:
                return False
        elif model.find("F7G") != -1:
            if version >= 7:
                return True
            else:
                return False
        else:
            return False

    def take_control_from_ec(self) -> None:
        """take over fan control from ec mcu"""
        if self.has_std_bios:
            return
        else:
            with open(self.fan_path + "gain", "w", encoding="utf8") as f:
                f.write(str(self.gain))
            with open(self.fan_path + "ramp_rate", "w", encoding="utf8") as f:
                f.write(str(self.ec_ramp_rate))
            with open(self.fan_path + "recalculate", "w", encoding="utf8") as f:
                f.write(str(1))

    def return_to_ec_control(self) -> None:
        """reset EC to generate fan values internally"""
        if self.has_std_bios:
            with open(self.fan_path + "fan1_target", "w", encoding="utf8") as f:
                f.write(str(int(0)))
        else:
            with open(self.fan_path + "gain", "w", encoding="utf8") as f:
                f.write(str(10))
            with open(self.fan_path + "ramp_rate", "w", encoding="utf8") as f:
                f.write(str(20))
            with open(self.fan_path + "recalculate", "w", encoding="utf8") as f:
                f.write(str(0))

    def get_speed(self) -> int:
        """returns the measured (real) fan speed"""
        with open(self.fan_path + "fan1_input", "r", encoding="utf8") as f:
            self.measured_speed = int(f.read().strip())
        return self.measured_speed

    def get_charge_state(self) -> bool:
        """updates min rpm depending on charge state"""
        with open(self.charge_state_path, "r", encoding="utf8") as f:
            state = f.read().strip()
        if state == "Charging":
            self.charge_state = True
        else:
            self.charge_state = False
        return self.charge_state

    def set_speed(self, speed) -> None:
        """sets a new target speed"""

        # overspeed commanded, set to max
        if speed > self.max_speed:
            speed = self.max_speed
        elif self.charge_state:
            if speed < self.charge_min_speed:
                speed = self.charge_min_speed
        elif speed < self.threshold_speed:
            if self.cold_off:
                speed = self.min_speed
            elif int(time.time()) - self.time_on >= self.min_time_on:
                speed = self.min_speed # if min_time satisfied, turn off
                self.cold_off = True
            else:
                speed = self.threshold_speed # else stay at threshold
        
        # make note of when fan is turned on
        if speed > self.min_speed and self.cold_off:
            self.time_on = int(time.time())
            self.cold_off = False
        self.fc_speed = speed
        with open(self.fan_path + "fan1_target", "w", encoding="utf8") as f:
            f.write(str(int(self.fc_speed)))


class Device:
    """devices are sources of heat - CPU, GPU, etc."""

    def __init__(self, base_path, config, fan_max_speed, n_sample_avg) -> None:
        """constructor"""
        self.sensor_path = (
            get_full_path(base_path, config["hwmon_name"]) + config["sensor_name"]
        )
        self.sensor_path_input = self.sensor_path + "_input"
        self.nice_name = config["nice_name"]
        self.max_temp = config["max_temp"]
        self.poll_reduction_multiple = config["poll_mult"]
        self.fan_max_speed = fan_max_speed
        self.n_sample_avg = n_sample_avg


        # try to pull critical temperature from the hwmon
        try:
            crit_temp = self.get_critical_temp()
            if not 60 <= crit_temp <= 95:
                raise Exception("critical temperature out of range")
            self.max_temp = crit_temp
            print(f"loaded critical temp from {self.nice_name} hwmon: {self.max_temp}")
        except:
            # print(f'failed to load critical temp from {self.nice_name} hwmon, falling back to config')
            pass

        # self.temp_deadzone = config["temp_deadzone"]
        self.temp_hysteresis = config["temp_hysteresis"]
        try:
            self.temp_threshold = config["T_threshold"]
        except:
            print(f"failed to load T_threshold")
            try:
                self.temp_threshold = config["T_setpoint"]
            except:
                print(f"failed to load T_setpoint")

        # state variables
        self.n_poll_requests = self.poll_reduction_multiple 
        self.measured_temp = self.get_temp()
        self.temps_buffer = deque([self.measured_temp] * self.n_sample_avg)
        self.avg_temp = 0
        self.control_temp = 0  # filtered temp, with hyseteresis, that is sent to controller to calculate output
        self.prev_control_temp = 0
        self.control_output = 0

        # instantiate controller depending on type
        self.type = config["type"]
        if self.type == "pid":
            self.controller = PID(
                float(config["Kp"]), float(config["Ki"]), float(config["Kd"])
            )
            self.controller.SetPoint = config["T_setpoint"]
            self.controller.setWindup(
                config["windup_limit"]
            )  # windup limits the I term of the output
        elif self.type == "quadratic":
            self.controller = Quadratic(
                float(config["A"]),
                float(config["B"]),
                float(config["C"]),
                float(config["T_threshold"]),
            )
        elif self.type == "feedforward":
            self.controller = FeedForward(
                float(config["Kp"]),
                float(config["Ki"]),
                float(config["Kd"]),
                int(config["windup"]),
                int(config["winddown"]),
                float(config["A_ff"]),
                float(config["B_ff"]),
                float(config["T_setpoint"]),
            )
        elif self.type == "ffmin":
            self.controller = FeedForwardMin(
                float(config["Kp"]),
                float(config["Ki"]),
                float(config["Kd"]),
                int(config["windup"]),
                int(config["winddown"]),
                float(config["A_ff"]),
                float(config["B_ff"]),
                float(config["T_setpoint"]),
                float(config["A_min"]),
                float(config["B_min"]),
            )
        elif self.type == "ffquad":
            self.controller = FeedForwardQuad(
                float(config["A_quad"]),
                float(config["B_quad"]),
                float(config["C_quad"]),
                float(config["A_ff"]),
                float(config["B_ff"]),
            )
        else:
            print("error loading device controller \n")
            exit(1)

    def get_critical_temp(self) -> float:
        """returns the critical temperature of the device"""
        with open(self.sensor_path + "_crit", "r", encoding="utf8") as f:
            return int(f.read().strip()) / 1000

    def get_temp(self) -> float:
        """updates temperatures"""
        self.n_poll_requests += 1
        if self.n_poll_requests >= self.poll_reduction_multiple:
            with open(self.sensor_path_input, "r", encoding="utf8") as f:
                temp = int(f.read().strip()) / 1000
                self.n_poll_requests = 0
                if temp >= 255:  # catch overflow
                    return self.temp_threshold
                else:
                    return temp
        return self.measured_temp

    def get_avg_temp(self) -> float:
        """updates temperature list + generates average value"""
        self.measured_temp = self.get_temp()
        self.temps_buffer.popleft()
        self.temps_buffer.append(self.measured_temp)
        self.avg_temp = math.fsum(self.temps_buffer) / self.n_sample_avg
        return self.avg_temp

    # update this to include hysteresis
    # TODO: define variables from config
    def get_output(self, power_input) -> int:
        """updates the device controller and returns bounded output"""
        if ( # check if temp is increasing, or has decreased past hysteresis
            self.avg_temp > self.prev_control_temp
            or self.prev_control_temp - self.avg_temp > self.temp_hysteresis
        ):
            self.control_temp = self.avg_temp
            self.prev_control_temp = self.control_temp

        self.controller.update(self.control_temp, power_input)
        self.control_output = max(self.controller.output, 0)
        if self.control_temp > self.max_temp:
            print(
                f"Warning: {self.nice_name} temperature of {self.control_temp} greater than max {self.max_temp}! Setting fan to max speed."
            )
            self.control_output = self.fan_max_speed
        return self.control_output


class Sensor:
    """sensor for measuring non-temperature values"""

    def __init__(self, base_path, config, t_fast, t_slow) -> None:
        self.sensor_path = (
            get_full_path(base_path, config["hwmon_name"]) + config["sensor_name"]
        )

        self.nice_name = config["nice_name"]
        self.power_threshold = config["low_power_threshold"]
        sensor_time_avg = config["sensor_time_avg"]
        self.n_avg_slow = int(sensor_time_avg / t_slow)
        self.n_avg_fast = int(sensor_time_avg / t_fast)

        self.avg_value = self.power_threshold #TODO should this start higher?
        self.is_low_power = True

        
        self.values_buffer = deque([self.avg_value] * self.n_avg_slow)

    def get_value(self) -> float:
        """returns instantaneous value"""
        with open(self.sensor_path, "r", encoding="utf-8") as f:
            value = int(f.read().strip()) / 1000000
        return value

    def get_avg_value(self) -> float:
        """returns average value"""
        self.value = self.get_value()
        if self.is_low_power and self.value > self.power_threshold:
            # low power state -> high power state
            self.is_low_power = False
            self.values_buffer = deque([self.avg_value] * (self.n_avg_fast - 1))
            self.values_buffer.append(self.value)
        elif not self.is_low_power and self.value <= self.power_threshold:
            # high power state -> low power state
            self.is_low_power = True
            self.values_buffer = deque([self.avg_value] * (self.n_avg_slow - 1))
            self.values_buffer.append(self.value)
        else:
            # pop oldest value and append latest reading
            self.values_buffer.popleft()
            self.values_buffer.append(self.value)

        self.avg_value = math.fsum(self.values_buffer) / len(self.values_buffer)
        return self.avg_value


def get_full_path(base_path, name) -> str:
    """helper function to find correct hwmon* path for a given device name"""
    for directory in os.listdir(base_path):
        full_path = base_path + directory + "/"
        try:
            test_name = open(full_path + "name", encoding="utf8").read().strip()
            if test_name == name:
                return full_path
        except:
            # print(f'failed to open {directory} folder for sensor {name}')
            pass
        # else:
        #    print(f'Sensor path for {name} was not found')

    raise FileNotFoundError(f"failed to find device {name}")


class FanController:
    """main FanController class"""

    def __init__(self, config_file, dmi: DmiId):
        """constructor"""
        # read in config yaml file
        with open(config_file, "r", encoding="utf8") as f:
            try:
                self.config = yaml.safe_load(f)
            except yaml.YAMLError as exc:
                print("error loading config file \n", exc)
                exit(1)

        # store global parameters
        self.base_hwmon_path = self.config["base_hwmon_path"]
        self.fast_loop_interval = self.config["fast_loop_interval"]
        self.slow_loop_interval = self.config["slow_loop_interval"]
        self.control_loop_ratio = self.config["control_loop_ratio"]
        self.log_write_ratio = self.config["log_write_ratio"]

        # initialize fan
        try:
            fan_path = get_full_path(
                self.base_hwmon_path, self.config["fan_hwmon_name"]
            )
        except FileNotFoundError:
            fan_path = get_full_path(
                self.base_hwmon_path, self.config["fan_hwmon_name_alt"]
            )
        finally:
            self.fan = Fan(fan_path, self.config, dmi)

        # initialize list of devices
        self.devices = [
            Device(
                self.base_hwmon_path,
                device_config,
                self.fan.max_speed,
                self.control_loop_ratio,
            )
            for device_config in self.config["devices"]
        ]

        # initialize APU power sensor
        self.power_sensor = Sensor(self.base_hwmon_path, self.config["sensors"][0], self.fast_loop_interval, self.slow_loop_interval)

        # exit handler
        signal.signal(signal.SIGTERM, self.on_exit)

    def print_single(self, source_name):
        """pretty print all device values, temp source, and output"""
        for device in self.devices:
            print(
                f"{device.nice_name}: {device.measured_temp:.1f}/{device.control_output:.0f}  ",
                end="",
            )
            # print("{}: {}  ".format(device.nice_name, device.measured_temp), end = '')
        print(
            f"{self.power_sensor.nice_name}: {self.power_sensor.value:.1f}/{self.power_sensor.avg_value:.1f}  ",
            end="",
        )
        print(f"Fan[{source_name}]: {int(self.fan.fc_speed)}/{self.fan.measured_speed}")

    def log_header(self):  
        header = ["TIMESTAMP"]
        for device in self.devices:
            header.append(f"{device.nice_name}_TEMP")
            header.append(f"{device.nice_name}_OUT")
        header.append(f"{self.power_sensor.nice_name}")
        header.append(f"{self.power_sensor.nice_name}_AVG")
        header.append(f"FAN_SRC")
        header.append(f"FAN_TARGET")
        header.append(f"FAN_REAL")
        self.log_writer.writerow(header)
        self.unwritten_log_lines = 1

    def log_single(self, source_name):
        row = [int(time.time())]
        for device in self.devices:
            row.append(int(device.measured_temp))
            row.append(int(device.control_output))
        row.append(f"{self.power_sensor.value:.2f}")
        row.append(f"{self.power_sensor.avg_value:.2f}")
        row.append(source_name)
        row.append(int(self.fan.fc_speed))
        row.append(self.fan.measured_speed)
        self.log_writer.writerow(row)
        self.unwritten_log_lines += 1
        if self.unwritten_log_lines >= self.log_write_ratio:
            self.log_file.flush()
            self.unwritten_log_lines = 0

    def loop_read_sensors(self):
        """internal loop to measure device temps and sensor value"""
        start_time = time.time()
        self.power_sensor.get_avg_value()
        for device in self.devices:
            device.get_avg_temp()

        # choose between low and high power loop interval
        loop_interval = self.slow_loop_interval if self.power_sensor.is_low_power else self.fast_loop_interval

        sleep_time = loop_interval - (time.time() - start_time)
        if sleep_time > 0:
            time.sleep(sleep_time)

    def loop_control(self):
        """main control loop"""
        # open log file
        log_file_path = "/var/log/jupiter-fan-control.log"
        old_log_file_path = "/var/log/jupiter-fan-control.old.log"

        try:
            # Check if the log file already exists, if it does, archive it
            if os.path.exists(log_file_path):
                if os.path.exists(old_log_file_path):
                    os.remove(old_log_file_path)
                os.rename(log_file_path, old_log_file_path)

            self.log_file = open(log_file_path, "w", encoding="utf8", newline="")
            # print(f'logging controller state to {log_file_path}')
            self.log_writer = csv.writer(self.log_file, delimiter=",")
            self.log_header()
        except Exception as e:
            print(f"unable to open log file {log_file_path} \n {e}")

        print("jupiter-fan-control started successfully.")
        while True:
            fan_error = abs(self.fan.fc_speed - self.fan.get_speed())
            if fan_error > 500:
                self.fan.take_control_from_ec()
            # read device temps and power sensor
            for _ in range(self.control_loop_ratio):
                self.loop_read_sensors()
                
            # read charge state
            self.fan.get_charge_state()
            for device in self.devices:
                device.get_output(self.power_sensor.avg_value)
            max_output = max(device.control_output for device in self.devices)
            self.fan.set_speed(max_output)
            # find source name for the max control output
            source_name = next(
                device for device in self.devices if device.control_output == max_output
            ).nice_name
            # print all values
            # self.print_single(source_name)
            # log all values
            try:
                self.log_single(source_name)
            except Exception as e:
                print(f'log single encountered error: {e}')
                pass

    def on_exit(self, signum, frame):
        """exit handler"""
        try:
            self.log_file.close()
        except:
            pass
        print("returning fan to EC control loop")
        self.fan.return_to_ec_control()
        exit()


# main
if __name__ == "__main__":
    dmi_id = DmiId()

    if dmi_id.board_name == "Jupiter":
        config_file_path = "/usr/share/jupiter-fan-control/jupiter-config.yaml"
    elif dmi_id.board_name == "Galileo":
        config_file_path = "/usr/share/jupiter-fan-control/galileo-config.yaml"
    else:
        raise NotImplementedError(
            f"DMI_ID Board Name not implemented! bios: {dmi_id.bios_version}    board: {dmi_id.board_name}"
        )

    # catch fan service trying to start before the hwmonitors are fully loaded
    for retry in range(10):
        try:
            controller = FanController(config_file=config_file_path, dmi=dmi_id)
            break
        except FileNotFoundError:  # delay for amdgpu late load
            print(f"Warning: hwmons not fully loaded, retrying...")
            time.sleep(0.2)
            continue
    if retry == 9:
        raise FileNotFoundError("Failed to load hwmons after 10 attempts.")

    args = sys.argv
    if len(args) == 2:
        command = args[1]
        if command == "--run":
            controller.loop_control()

    # otherwise, exit cleanly
    controller.on_exit(None, None)
