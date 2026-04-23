#!/usr/bin/python -u
"""jupiter-fan-controller"""
import csv
import math
import os
import signal
import sys
import time
from collections import deque
from pathlib import Path

import yaml


class Dummy:
    def __init__(self) -> None:
        self.output = 0

    def update(self, temp_input, _) -> int:
        return 0


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


class FeedForwardQuad:
    """FF with an additional min curve"""

    def __init__(self, a_quad, b_quad, c_quad, a_ff, b_ff) -> None:
        """constructor"""
        self.a_ff = a_ff
        self.b_ff = b_ff
        self.quad = Quadratic(a_quad, b_quad, c_quad)
        self.output = 0

    def get_ff_setpoint(self, power_input) -> int:
        """returns the feed forward portion of the controller output"""
        return int(self.a_ff * power_input + self.b_ff)

    def update(self, temp_input, power_input) -> int:
        """run controller to update output"""
        quad_output = int(self.quad.update(temp_input, None))
        ff_output = self.get_ff_setpoint(power_input)
        self.output = quad_output + ff_output
        return self.output


class DmiId:
    def __init__(self) -> None:
        self.id = Path("/sys/class/dmi/id")
        self.bios_version = self.read("bios_version")
        self.board_name = self.read("board_name")

    def read(self, identifier):
        try:
            return open(self.id / identifier, encoding="utf-8").read().strip()
        except FileNotFoundError:
            return None


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
        self.fan_hysteresis = config["fan_hysteresis"]
        self.fc_speed = 0
        self.measured_speed = 0
        self.nohyst_speed = 0
        self.time_on = 0
        self.cold_off = True
        self.charge_state = False
        self.charge_min_speed = self.threshold_speed
        self.has_std_bios = self.bios_compatibility_check(dmi)
        self._reset_hysteresis()
        self.take_control_from_ec()
        self.set_speed(self.threshold_speed)

    @staticmethod
    def bios_compatibility_check(dmi: DmiId) -> bool:
        """returns True for bios versions >= 106, false for earlier versions"""
        try:
            model = dmi.bios_version[0:3]
            version = int(dmi.bios_version[3:7])
        except ValueError:
            print(f'Compatibility Check Skipped! DmiId bios_version:{dmi.bios_version} board_name:{dmi.board_name}')
            return True

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
        elif model.find("F7F") != -1:
                return True
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
                f.write(str(0))
        else:
            with open(self.fan_path + "gain", "w", encoding="utf8") as f:
                f.write(str(10))
            with open(self.fan_path + "ramp_rate", "w", encoding="utf8") as f:
                f.write(str(20))
            with open(self.fan_path + "recalculate", "w", encoding="utf8") as f:
                f.write(str(0))

    def get_speed(self) -> int:
        """returns the measured (real) fan speed"""
        with open(self.fan_path + "fan1_input", encoding="utf8") as f:
            self.measured_speed = int(f.read().strip())
        return self.measured_speed

    def get_charge_state(self) -> bool:
        """updates min rpm depending on charge state"""
        if self.charge_state_path is False:
            return False
        with open(self.charge_state_path, encoding="utf8") as f:
            state = f.read().strip()
        if state == "Charging":
            self.charge_state = True
        else:
            self.charge_state = False
        return self.charge_state

    def _reset_hysteresis(self):
        self._hyst_min = 0
        self._hyst_max = 0

    def _apply_hysteresis(self, new_speed: float) -> float:
        """Applies hysteresis filtering on fan output."""
        if new_speed > self._hyst_max:
            self._hyst_max = new_speed
            self._hyst_min = new_speed - self.fan_hysteresis
            return new_speed
        elif new_speed < self._hyst_min:
            self._hyst_min = new_speed
            self._hyst_max = new_speed + self.fan_hysteresis
            return self._hyst_max
        else:
            return self.fc_speed

    def set_speed(self, speed) -> None:
        """sets a new target speed"""

        # overspeed commanded, set to max
        if speed > self.max_speed:
            speed = self.max_speed

        # apply output hysteresis
        self.nohyst_speed = speed
        speed = self._apply_hysteresis(speed)

        # bound speed by minimum, taking into account charge state
        if self.charge_state:
            if speed < self.charge_min_speed:
                speed = self.charge_min_speed
        elif self.min_time_on > 0 and speed < self.threshold_speed:
            if self.cold_off:
                speed = self.min_speed
            elif int(time.time()) - self.time_on >= self.min_time_on:
                speed = self.min_speed
                self.cold_off = True
            else:
                speed = self.threshold_speed
        elif speed <= self.min_speed:
            speed = self.min_speed

        if self.min_time_on > 0 and speed > self.min_speed and self.cold_off:
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
        except Exception:
            pass

        self.temp_hysteresis = config["temp_hysteresis"]
        self.temp_threshold = config.get("T_threshold", 0)

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
        if self.type == "quadratic":
            self.controller = Quadratic(
                float(config["A"]),
                float(config["B"]),
                float(config["C"]),
                float(config["T_threshold"]),
            )
        elif self.type == "ffquad":
            self.controller = FeedForwardQuad(
                float(config["A_quad"]),
                float(config["B_quad"]),
                float(config["C_quad"]),
                float(config["A_ff"]),
                float(config["B_ff"]),
            )
        elif self.type == "dummy":
            self.controller = Dummy()
        else:
            print("error loading device controller \n")
            exit(1)

    def get_critical_temp(self) -> float:
        """returns the critical temperature of the device"""
        with open(self.sensor_path + "_crit", encoding="utf8") as f:
            return int(f.read().strip()) / 1000

    def get_temp(self) -> float:
        """updates temperatures"""
        self.n_poll_requests += 1
        if self.n_poll_requests >= self.poll_reduction_multiple:
            with open(self.sensor_path_input, encoding="utf8") as f:
                try:
                    temp = int(f.read().strip()) / 1000
                except PermissionError:
                    temp = 0
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

    def get_output(self, power_input) -> int:
        """updates the device controller and returns bounded output"""
        if (
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

        try:
            self.avg_value = self.get_value()
        except Exception as e:
            print(f'Sensor initialization: get_value() returned {e}')
            self.avg_value = self.power_threshold
        self.is_low_power = True

        self.values_buffer = deque([self.avg_value] * self.n_avg_slow)

    def get_value(self) -> float:
        """returns instantaneous value"""
        with open(self.sensor_path, encoding="utf-8") as f:
            try:
                value = int(f.read().strip()) / 1000000
            except PermissionError:
                value = 0
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
        except Exception:
            pass

    raise FileNotFoundError(f"failed to find device {name}")


class FanController:
    """main FanController class"""
    LOG_FILE_PATH = Path("/var/log/jupiter-fan-control.log")
    LOG_FILE_MAX_SIZE = 2**20

    def __init__(self, config_file, dmi: DmiId):
        """constructor"""
        # read in config yaml file
        with open(config_file, encoding="utf8") as f:
            try:
                self.config = yaml.safe_load(f)
            except yaml.YAMLError as exc:
                print("error loading config file \n", exc)
                exit(1)

        # store global parameters from config
        self.base_hwmon_path = self.config["base_hwmon_path"]
        self.fast_loop_interval = self.config["fast_loop_interval"]
        self.slow_loop_interval = self.config["slow_loop_interval"]
        self.control_loop_ratio = self.config["control_loop_ratio"]
        self.log_write_ratio = self.config["log_write_ratio"]

        self.initialize_fan(dmi)

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

        # initialize APU power sensor #TODO make this work with all hardware types, consider adding RAPLSensor from tuner.py
        self.power_sensor = Sensor(
            self.base_hwmon_path,
            self.config["sensors"][0],
            self.fast_loop_interval,
            self.slow_loop_interval,
        )

        # exit handler
        signal.signal(signal.SIGTERM, self.on_exit)

    def initialize_fan(self, dmi: DmiId):
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

    def print_single(self, source_name):
        """pretty print all device values, temp source, and output"""
        for device in self.devices:
            print(
                f"{device.nice_name}: {device.measured_temp:.1f}/{device.control_output:.0f}  ",
                end="",
            )
        print(
            f"{self.power_sensor.nice_name}: {self.power_sensor.value:.1f}/{self.power_sensor.avg_value:.1f}  ",
            end="",
        )
        print(f"Fan[{source_name}]: {int(self.fan.fc_speed)}/{self.fan.measured_speed}")

    def initialize_log_file(self):
        try:
            # Check if the log file already exists, if it does, rotate it
            if self.LOG_FILE_PATH.exists():
                self.LOG_FILE_PATH.rename(self.LOG_FILE_PATH.with_suffix('.old.log'))

            self.log_file = open(self.LOG_FILE_PATH, "w", encoding="utf8", newline="")
            self.log_writer = csv.writer(self.log_file, delimiter=",")
            self.log_rows_buffer = []
            self.log_header()
        except Exception as e:
            print(f'failed to initialize log file: {e}')

    def log_header(self):
        header = ["TIMESTAMP"]
        for device in self.devices:
            header.append(f"{device.nice_name}_TEMP")
            header.append(f"{device.nice_name}_OUT")
        header.append(f"{self.power_sensor.nice_name}")
        header.append(f"{self.power_sensor.nice_name}_AVG")
        header.append("FAN_SRC")
        header.append("FAN_TARGET")
        header.append("FAN_REAL")
        self.log_writer.writerow(header)

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
        self.log_rows_buffer.append(row)
        self.flush_or_rotate_log_if_needed()

    def flush_or_rotate_log_if_needed(self):
        if self.log_file.tell() >= self.LOG_FILE_MAX_SIZE:
            print('Maximum size reached, rotating log')
            self.log_writer.writerows(self.log_rows_buffer)
            self.log_file.close()
            self.initialize_log_file()
        elif len(self.log_rows_buffer) >= self.log_write_ratio:
            self.log_writer.writerows(self.log_rows_buffer)
            self.log_file.flush()
            self.log_rows_buffer = []

    def loop_read_sensors(self):
        """internal loop to measure device temps and sensor value"""
        start_time = time.time()
        self.power_sensor.get_avg_value()
        for device in self.devices:
            device.get_avg_temp()

        # choose between low and high power loop interval
        loop_interval = (
            self.slow_loop_interval
            if self.power_sensor.is_low_power
            else self.fast_loop_interval
        )

        sleep_time = loop_interval - (time.time() - start_time)
        if sleep_time > 0:
            time.sleep(sleep_time)

    def loop_control(self):
        """main control loop"""

        self.initialize_log_file()
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
            source_name = next(
                device for device in self.devices if device.control_output == max_output
            ).nice_name
            try:
                self.log_single(source_name)
            except Exception as e:
                print(f"log single encountered error: {e}")

    def on_exit(self, signum, frame):
        """exit handler"""
        try:
            if len(self.log_rows_buffer) > 0:
                self.log_writer.writerows(self.log_rows_buffer)
            self.log_file.close()
            print("closed log file")
        except Exception:
            pass
        print("returning fan to EC control loop")
        self.fan.return_to_ec_control()
        exit()


# main
if __name__ == "__main__":
    dmi_id = DmiId()
    script_dir = Path(__file__).resolve().parent

    if dmi_id.board_name == "Jupiter":
        config_file_path = script_dir / "jupiter-config.yaml"
    elif dmi_id.board_name == "Galileo":
        config_file_path = script_dir / "galileo-config.yaml"
    elif dmi_id.board_name == "Fremont":
        config_file_path = script_dir / "fremont-config.yaml"
    else:
        sys.exit(0)

    # catch fan service trying to start before the hwmonitors are fully loaded
    for retry in range(10):
        try:
            controller = FanController(config_file=config_file_path, dmi=dmi_id)
            break
        except FileNotFoundError:  # delay for amdgpu late load
            print("Warning: hwmons not fully loaded, retrying...")
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
