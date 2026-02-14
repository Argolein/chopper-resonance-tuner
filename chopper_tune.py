"""Chopper Tune extension for Klipper.

TMC drivers registers calibration tool.

Copyright (C) 2024  Alexander Fedorov <altzbox@gmail.com>
Copyright (C) 2024  Maksim Bolgov <maksim8024@gmail.com>

This file may be distributed under the terms of the GNU GPLv3 license.
"""

# Standard Library Imports
from __future__ import annotations

import json
import math
import os
import platform
import statistics
import traceback
from datetime import datetime
from enum import IntEnum
from functools import reduce
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import sys
    from types import TracebackType

    from configfile import ConfigWrapper
    from extras.adxl345 import Accel_Measurement
    from gcode import GCodeCommand, GCodeDispatch
    from klippy import Printer
    from reactor import PollReactor
    from toolhead import ToolHead

    if sys.version_info >= (3, 11):
        from typing import Self
    else:
        from typing_extensions import Self


DEFAULT_ACCEL_CHIP = "adxl345"

# Snapmaker U1 policy: all artifacts must live under this folder.
U1_LOG_ROOT = "/data/gcodes/chopper-tuner"

# Backwards-compat aliases for legacy code paths (kept non-/tmp).
RESULTS_FOLDER = U1_LOG_ROOT
DATA_FOLDER = os.path.join(U1_LOG_ROOT, "tmp")

FCLK = 12  # MHz
CUTOFF_RANGE = 5
# Graphs generation
COLORS = [
    "",
    "#2F4F4F",
    "#12B57F",
    "#9DB512",
    "#DF8816",
    "#1297B5",
    "#5912B5",
    "#B51284",
    "#127D0C",
]


class MeasurementMode(IntEnum):
    """Integer enumerator to specify the current measurement mode."""

    Resonances = 0
    Vibrations = 1

    def __repr__(self) -> str:
        """Return the enum name for str().

        Returns:
            str: The name as the string representation of this MeasurementMode.
        """
        return self.name

    __str__ = __repr__

    @classmethod
    def to_mode(cls, mode: int | str | MeasurementMode) -> MeasurementMode:
        """Convert the given mode value to a MeasurementMode enum.

        Args:
            mode (int | str | MeasurementMode]): The value to convert to a
                MeasurementMode.

        Raises:
            TypeError: Input value type is invalid.
            ValueError: Input value is invalid.

        Returns:
            MeasurementMode: The enum.
        """
        if not isinstance(mode, (int, str, MeasurementMode)):
            raise TypeError(
                "mode should be a MeasurementMode enum value or one of "
                f"{[m.name for m in cls] + [m.value for m in cls]}, "
                f"not {mode.__class__.__name__}: '{mode}'"
            )
        if isinstance(mode, str):
            mode_name_lut = {m.name.lower(): m.name for m in cls}
            mode_name_lut.update({m.value: m.name for m in cls})
            mode_lower_case = mode.lower()
            if mode_lower_case not in mode_name_lut:
                raise ValueError(
                    "mode should be a MeasurementMode enum value or one of "
                    f"{[m.name for m in cls] + [m.value for m in cls]}, "
                    f"not '{mode}'"
                )

            return cls.__members__[mode_name_lut[mode_lower_case]]

        return mode


class SearchMethod(IntEnum):
    """Integer enumerator to specify the current search method."""

    BruteForce = 0
    Adaptive = 1
    Progressive = 2

    def __repr__(self) -> str:
        """Return the enum name for str().

        Returns:
            str: The name as the string representation of this SearchMethod.
        """
        return self.name

    __str__ = __repr__

    @classmethod
    def to_method(cls, method: int | str | SearchMethod) -> SearchMethod:
        """Convert the given method value to a SearchMethod enum.

        Args:
            method (int | str | SearchMethod]): The value to convert to a
                SearchMethod.

        Raises:
            TypeError: Input value type is invalid.
            ValueError: Input value is invalid.

        Returns:
            SearchMethod: The enum.
        """
        if not isinstance(method, (int, str, SearchMethod)):
            raise TypeError(
                "method should be a SearchMethod enum value or one of "
                f"{[m.name for m in cls] + [m.value for m in cls]}, "
                f"not {method.__class__.__name__}: '{method}'"
            )
        if isinstance(method, str):
            method_name_lut = {m.name.lower(): m.name for m in cls}
            method_name_lut.update({m.value: m.name for m in cls})
            method_lower_case = method.lower()
            if method_lower_case.replace("_", "") not in method_name_lut:
                raise ValueError(
                    "method should be a SearchMethod enum value or one of "
                    f"{[m.name for m in cls] + [m.value for m in cls]}, "
                    f"not '{method}'"
                )

            return cls.__members__[method_name_lut[method_lower_case.replace("_", "")]]

        return method


class AccelerometerMeasure:
    """A context manager that helps with accelerometer measurement."""

    def __init__(
        self,
        accel_chip: object,
    ) -> None:
        self.accel_chip = accel_chip
        self.bg_client = None
        self.samples: None | list[Accel_Measurement] = None

    def __enter__(self) -> Self:
        """Enter to the context."""
        if self.bg_client is None:
            # Both adxl345 and lis2dw on the U1 support this API.
            self.bg_client = self.accel_chip.start_internal_client()
        return self

    def __exit__(
        self,
        exc_type: None | type[BaseException],
        exc_value: None | BaseException,
        tb: None | TracebackType,
    ) -> None:
        """Exit the context.

        Ignore the exceptions, if any, Klipper will handle it.
        """
        self.bg_client.finish_measurements()
        # retrieve data
        self.samples = self.bg_client.samples or self.bg_client.get_samples()


class Coord(list):
    """Custom "list" class for coordinates - add easy access to x, y, z components.

    The difference between the gcode.Coord is that, this class allows attribute
    setting.

    Args:
        t (Coord | list | tuple): Another Coord instance or a list or a tuple.
    """

    __slots__ = ()

    def __new__(cls, t: Coord | list | tuple) -> Self:
        """Create a new Coord instance."""
        if len(t) < 4:
            t = list(tuple(t) + (0,) * (3 - len(t)))
        return list.__new__(cls, t)

    @property
    def x(self) -> float:
        """Return the x component.

        Returns:
            float: The x component.
        """
        return self[0]

    @x.setter
    def x(self, x: float) -> None:
        """Set the x component.

        Args:
            x (float): The x component value.
        """
        self[0] = x

    @property
    def y(self) -> float:
        """Return the y component.

        Returns:
            float: The y component.
        """
        return self[1]

    @y.setter
    def y(self, y: float) -> None:
        """Set the y component.

        Args:
            y (float): The y component value.
        """
        self[1] = y

    @property
    def z(self) -> float:
        """Return the z component.

        Returns:
            float: The z component.
        """
        return self[2]

    @z.setter
    def z(self, z: float) -> None:
        """Set the z component.

        Args:
            z (float): The z component value.
        """
        self[2] = z

    def length(self) -> float:
        """Return the vector length.

        Returns:
            float: The vector length.
        """
        return float(reduce(lambda x, y: x + y**2, [0, *self]) ** 0.5)

    def unitize(self) -> Self:
        """Make self unit vector."""
        other = self / self.length()
        for i in range(len(self)):
            self[i] = other[i]
        return self

    def __add__(self, other: Coord | list | tuple | float) -> Coord:
        """Overload the + operator.

        Args:
            other (Coord | list | tuple | float): The other operand.

        Returns:
            Coord: The result of the addition.
        """
        if isinstance(other, (Coord, list, tuple)):
            return Coord((self.x + other[0], self.y + other[1], self.z + other[2]))
        if isinstance(other, (int, float)):
            return Coord([i + other for i in self])
        return super().__add__(other)

    def __iadd__(self, other: Coord | list | tuple | float) -> Self:
        """Overload the += operator.

        Args:
            other (Coord | list | tuple | float): The other operand.

        Returns:
            Self: The result of the addition.
        """
        if isinstance(other, (Coord, list, tuple)):
            return Coord((self.x + other[0], self.y + other[1], self.z + other[2]))
        if isinstance(other, (int, float)):
            return Coord([i + other for i in self])
        return super().__iadd__(other)

    def __sub__(self, other: Coord | list | tuple | float) -> Coord:
        """Overload the - operator.

        Args:
            other (Coord | list | tuple | float): The other operand.

        Returns:
            Coord: The result of the addition.
        """
        if isinstance(other, (Coord, list, tuple)):
            return Coord((self.x - other[0], self.y - other[1], self.z - other[2]))
        if isinstance(other, (int, float)):
            return Coord([i - other for i in self])
        return super().__sub__(other)

    def __mul__(self, other: Coord | list | tuple | float) -> Coord:
        """Overload the * operator.

        Args:
            other (Coord | list | tuple | float): The other operand.

        Returns:
            Coord: The result of the multiplication.
        """
        if isinstance(other, (Coord, list, tuple)):
            return Coord((self.x * other[0], self.y * other[1], self.z * other[2]))
        if isinstance(other, (int, float)):
            return Coord([i * other for i in self])
        return super().__mul__(other)

    def __imul__(self, other: Coord | list | tuple | float) -> Self:
        """Overload the *= operator.

        Args:
            other (Coord | list | tuple | float): The other operand.

        Returns:
            Self: The result of the multiplication.
        """
        if isinstance(other, (Coord, list, tuple)):
            return Coord((self.x * other[0], self.y * other[1], self.z * other[2]))
        if isinstance(other, (int, float)):
            return Coord([i * other for i in self])
        return super().__imul__(other)

    def __truediv__(self, other: Coord | list | tuple | float) -> Coord:
        """Overload the / operator.

        Args:
            other (Coord | list | tuple | float): The other operand.

        Returns:
            Coord: The result of the division.
        """
        if isinstance(other, (Coord, list, tuple)):
            return Coord((self.x / other[0], self.y / other[1], self.z / other[2]))
        if isinstance(other, (int, float)):
            return Coord([i / other for i in self])
        return super().__truediv__(other)

    def __itruediv__(self, other: Coord | list | tuple | float) -> Self:
        """Overload the /= operator.

        Args:
            other (Coord | list | tuple | float): The other operand.

        Returns:
            Coord: The result of the division.
        """
        if isinstance(other, (Coord, list, tuple)):
            return Coord((self.x / other[0], self.y / other[1], self.z / other[2]))
        if isinstance(other, (int, float)):
            return Coord([i / other for i in self])
        return super().__itruediv__(other)


class CoordGenerator:
    """A class to generate coordinates/positions for chopper tuning.

    Args:
        direction (Coord | list | tuple): The initial direction.
        start_coord (None | Coord): The starting coordinate.
    """

    def __init__(
        self, direction: Coord | list | tuple, start_coord: None | Coord = None
    ) -> None:
        self._direction = None
        self.direction = direction
        if start_coord is None:
            start_coord = Coord((0, 0, 0))
        self.current_coord = start_coord

    @property
    def direction(self) -> Coord:
        """Return the direction of this CoordGenerator.

        Returns:
            Coord: The direction of this CoordGenerator.
        """
        return self._direction

    @direction.setter
    def direction(self, direction: Coord | list | float) -> None:
        """Set the direction attribute value.

        Args:
            direction (Coord | list | float): The direction value.

        Raises:
            TypeError: If the direction is not a list, tuple or Coord instance.
        """
        if not isinstance(direction, (list, tuple, Coord)):
            raise TypeError(
                f"direction should be a list, tuple or a Coord instance, not "
                f"{direction.__class__.__name__}: '{direction}'"
            )

        if isinstance(direction, (list, tuple)):
            direction = Coord(direction)

        if direction.length() == 0:
            raise ValueError("direction length cannot be zero.")

        self._direction = direction.unitize()

    def switch_direction(self) -> None:
        """Switch direction."""
        self.direction *= (-1, -1, -1)

    def next(self, travel_distance: float) -> float:
        """Get the next position.

        Args:
            travel_distance (float): The travel distance.

        Returns:
            float: The next position.
        """
        self.current_coord += self.direction * travel_distance
        return self.current_coord


class ChopperTune:
    """The main class to handle the chopper tune functionality.

    Args:
        config (ConfigWrapper): The configuration wrapper.
    """

    def __init__(self, config: ConfigWrapper) -> None:
        self.printer: Printer = config.get_printer()
        self.gcode: GCodeDispatch = self.printer.lookup_object("gcode")
        self.configfile = self.printer.lookup_object("configfile")
        self.toolhead: None | ToolHead = None
        self.settings = None
        self.reactor: PollReactor = self.printer.get_reactor()
        self.driver_settings = {}
        self.stepper_settings = {}
        self.registers = {
            "stepper_count": 0,
            "tbl": -1,
            "toff": -1,
            "hend": -1,
            "hstrt": -1,
            "tpfd": -1,
            "curr": -1,
        }

        # config values
        self.debug = config.getboolean("debug", False)
        self.inset = config.getfloat("inset", 10)
        self.current_change_step = config.getint("current_change_step", 25)
        self.measure_time = config.getint("measure_time", 1250)
        self.required_rpm = config.getfloatlist("required_rpm", [37.5, 150, 1.5])
        self.delay = config.getfloat("delay", 500)
        self.fclk = config.getint("fclk", 12)

        self.kinematics = config.getsection("printer").get("kinematics")

        # runtime variables
        self.driver = None
        self.resistor = None
        self.samples = {}  # store all samples
        self.total_expected_samples = -1  # the expected number of samples
        self.number_of_samples = 0  # current number of samples taken
        self.number_of_real_samples = 0  # current number of real samples taken
        self.best_result = 999_999_999  # store the best result
        self.speed_vs_vibrations = []  # store speed vs vibrations
        self.global_start_time = 0  # store the global start time

        # Calculated values
        self.search_method = None
        self.measurement_mode = MeasurementMode.Vibrations
        self.travel_speed = None
        self.travel_distance = None
        self.coord_generator = None
        self.accel_chip = None
        self.steppers = None
        self.current = None
        self.static_noise_vector = None
        self.static_noise_magnitude = None
        self.initial_position = None
        self.initial_direction = None

        # Bounds
        self.bounds = []
        self.current_min = None
        self.current_max = None
        self.tbl_min = None
        self.tbl_max = None
        self.toff_min = None
        self.toff_max = None
        self.hstrt_min = None
        self.hstrt_max = None
        self.hstrt_hend_max = None
        self.hend_min = None
        self.hend_max = None
        self.tpfd_min = None
        self.tpfd_max = None
        self.min_speed = None
        self.max_speed = None
        self.speed_change_step = None

        self.register_commands()
        self.printer.register_event_handler("klippy:connect", self._connect)

    def register_commands(self) -> None:
        """Register GCode commands."""
        self.gcode.register_command("CHOPPER_TUNE", self.cmd_chopper_tune)
        self.gcode.register_command("CHOPPER_TUNE_DEBUG", self.cmd_chopper_tune_debug)

    def _connect(self) -> None:
        """Handle printer connect event."""
        self.settings = self.configfile.get_status(None)["settings"]
        self.toolhead = self.printer.lookup_object("toolhead")
        for axis in "xyz":
            driver, _ = self.detect_driver(axis)
            self.driver_settings[f"stepper_{axis}"] = self.settings.get(
                f"tmc{driver} stepper_{axis}", {}
            )
            self.stepper_settings[f"stepper_{axis}"] = self.settings.get(
                f"stepper_{axis}", {}
            )
        # Do not resolve the accelerometer here; on U1 we use LIS2DW on Tool 0
        # and will resolve it dynamically from [resonance_tester].

    def detect_driver(self, stepper: str) -> None | tuple[str, str]:
        """Detect the driver of the selected stepper.

        Args:
            stepper (str): The stepper name.

        Returns:
            tuple[None, None] | tuple[str, str]: A tuple containing the stepper
                driver model and sense resistor, or None if the driver thus the
                sense resistor cannot be detected.
        """
        drivers = ["2130", "2208", "2209", "2660", "2240", "5160"]
        stepper = f"stepper_{stepper}"
        resistor = None
        for driver in drivers:
            if "run_current" not in self.settings.get(f"tmc{driver} {stepper}", {}):
                continue
            self.gcode.respond_info(f"Selected tmc{driver} for {stepper}")
            if driver != "2240":
                resistor = self.settings[f"tmc{driver} {stepper}"]["sense_resistor"]
            else:
                resistor = self.settings[f"tmc{driver} {stepper}"]["rref"]
            return driver, resistor

        return None, None

    def get_axes_and_steppers(
        self, axis: str
    ) -> tuple[tuple[str, ...], tuple[str, ...]]:
        """Get main and secondary axis / stepper.

        Args:
            axis (str): The to be tuned.

        Returns:
            tuple[tuple[str, ...], tuple[str, ...]]: A tuple containing:
                - axes (tuple[str, ...]): The main and secondary axis.
                - steppers (tuple[str, ...]): The main and secondary stepper.
        """
        if axis not in ("x", "y", "z"):
            raise self.printer.command_error(f"WARNING!!! Incorrect axis: {axis}")

        if self.kinematics not in ("corexy", "cartesian"):
            raise self.printer.command_error(
                f"WARNING!!! Unsupported kinematics: {self.kinematics}"
            )

        if self.kinematics == "corexy":
            if axis in ("x", "y"):
                if axis == "x":
                    axes = ("x", "y")
                    steppers = ("stepper_x", "stepper_y")
                elif axis == "y":
                    axes = ("y", "x")
                    steppers = ("stepper_y", "stepper_x")
            elif axis == "z":
                axes = ("z", "x")
                steppers = ("stepper_z",)
        elif self.kinematics == "cartesian":
            if axis == "x":
                axes = ("x", "y")
                steppers = ("stepper_x",)
            elif axis == "y":
                axes = ("y", "x")
                steppers = ("stepper_y",)
            elif axis == "z":
                axes = ("z", "x")
                steppers = ("stepper_z",)

        return axes, steppers

    def get_axis_limits(self, axes: list[str]) -> tuple[float, float, float, float]:
        """Select main and secondary axis / stepper.

        Args:
            axes (list[str]): The main and secondary axis.

        Returns:
            tuple[float, float, float, float]: A tuple containing:
                - a_axis_min (float): The minimum position of the main axis.
                - a_axis_max (float): The maximum position of the main axis.
                - a_axis_mid (float): The middle position of the main axis.
                - b_axis_mid (float): The middle position of the secondary axis.
        """
        if axes[0] == "z":
            a_axis_min = (
                max(self.stepper_settings[f"stepper_{axes[0]}"]["position_min"], 0)
                + self.inset
            )
        else:
            a_axis_min = (
                self.stepper_settings[f"stepper_{axes[0]}"]["position_min"] + self.inset
            )

        a_axis_max = (
            self.stepper_settings[f"stepper_{axes[0]}"]["position_max"] - self.inset
        )
        a_axis_mid = (
            self.stepper_settings[f"stepper_{axes[0]}"]["position_max"]
            + self.stepper_settings[f"stepper_{axes[0]}"]["position_min"]
        ) / 2
        b_axis_mid = (
            self.stepper_settings[f"stepper_{axes[1]}"]["position_max"]
            + self.stepper_settings[f"stepper_{axes[1]}"]["position_min"]
        ) / 2
        return a_axis_min, a_axis_max, a_axis_mid, b_axis_mid

    def get_travel_speed_and_acceleration(self, axes: list[str]) -> tuple[float, float]:
        """Select main and secondary axis / stepper.

        Args:
            axes (list[str]): The main and secondary axis.

        Returns:
            tuple[float, float]: A tuple containing:
                - acceleration (float): The acceleration for the movement.
                - travel_speed (float): The travel speed for idle movements.
        """
        if axes[0] == "z":
            acceleration = self.settings["printer"]["max_z_accel"]
            # Idle movements speed
            travel_speed = self.settings["printer"].get("max_z_velocity", 0) / 2
        else:
            acceleration = self.settings["printer"].get("max_accel")
            # Idle movements speed
            travel_speed = self.settings["printer"].get("max_velocity") / 2

        return acceleration, travel_speed

    def validate_tpfd_values(
        self,
        driver: str,
        tpfd_min: int,
        tpfd_max: int,
    ) -> None:
        """Validate the TPFD min and max values.

        Args:
            driver (str): The stepper driver model.
            tpfd_min (int): The TPFD min value.
            tpfd_max (int): The TPFD max value.
        """
        if tpfd_min != -1 or tpfd_max != -1:
            if driver in ["2240", "5160"]:
                if tpfd_min < 0 or tpfd_max < 0:
                    self.printer.command_error("WARNING!!! Incorrect TPFD values")
            else:
                self.printer.command_error(
                    f"WARNING!!! TMC{driver} don't support register TPFD"
                )

    def reset_sample_data(self) -> None:
        """Reset sample data."""
        self.total_expected_samples = -1
        self.number_of_samples = 0
        self.number_of_real_samples = 0
        self.best_result = 999_999_999
        self.samples = {}
        self.speed_vs_vibrations = []
        self.global_start_time = 0

    def reset_registers(self) -> None:
        """Reset registers to default values."""
        # Reset register values
        self.registers = {
            "stepper_count": 0,
            "tbl": -1,
            "toff": -1,
            "hend": -1,
            "hstrt": -1,
            "tpfd": -1,
            "curr": -1,
        }

    def apply_registers(
        self,
        steppers: list[str],
        field: str,
        value: int,
    ) -> None:
        """Apply registers.

        Args:
            steppers (list[str]): The name of the steppers to set the register
                field values of.
            field (str): The name of the field to set the value of.
            value (int): The value to set to.
        """
        if field is None or value is None:
            return

        stepper = steppers[0]  # just update the main stepper
        for stepper_index in range(self.registers["stepper_count"]):
            # stepper_x,
            # stepper_y,
            # stepper_z, stepper_z1, stepper_z2, stepper_z3, ...
            # don't add index for the first stepper
            stepper_index = str(stepper_index) if stepper_index > 0 else ""
            if self.debug:
                self.gcode.respond_info(
                    f"Setting {field.lower()} "
                    f"from {self.registers[field]} to {value} "
                    f"on {stepper}{stepper_index}"
                )

            if field.lower() == "curr":
                if self.registers[field.lower()] != value:
                    self.gcode.run_script_from_command(
                        f"SET_TMC_CURRENT STEPPER={stepper} CURRENT={value / 1000}"
                    )
            elif (
                not (field == "tpfd" and value == -1)
                and self.registers[field.lower()] != value
            ):
                self.gcode.run_script_from_command(
                    "SET_TMC_FIELD "
                    f"STEPPER={stepper}{stepper_index} "
                    f"FIELD={field} VALUE={value}"
                )
        # store the last applied value
        self.registers[field.lower()] = value

    def get_stepper_count(self, axis: str) -> int:
        """Get the stepper count of the given axis.

        Args:
            axis (str): One of ["x", "y", "z"].

        Returns:
            int: The stepper count of the requested axis.
        """
        axis_steppers = [
            key for key in self.settings if key.startswith(f"stepper_{axis}")
        ]
        return len(axis_steppers)

    def get_accelerometer_chip(self, accel_chip: str) -> str:
        """Get accelerometer chip.

        Args:
            accel_chip (str): Accelerometer chip name, i.e adxl345.
        """
        # Select accelerometer
        if accel_chip == "default":
            resonance_tester = self.settings.get("resonance_tester", {})
            if "accel_chip" in resonance_tester:
                accel_chip = resonance_tester["accel_chip"]
            else:
                # Use Default accelerometer
                accel_chip = DEFAULT_ACCEL_CHIP

        self.gcode.respond_info(f"Selected {accel_chip} for accelerometer")
        return accel_chip

    def get_current_range(
        self,
        measurement_mode: MeasurementMode,
        current_min: None | int,
        current_max: None | int,
        steppers: list[str],
    ) -> tuple[int, int]:
        """Get run current.

        Args:
            measurement_mode (MeasurementMode): The measurement mode.
            current_min (None | int): The minimum current value.
            current_max (None | int): The maximum current value.
            steppers (list[str]): The main and secondary stepper.

        Returns:
            tuple[int, int]: The minimum and maximum current values.
        """
        # Select run_current
        run_current = int(
            float(self.driver_settings[steppers[0]].get("run_current")) * 1000
        )
        if current_min is None:
            current_min = run_current
            if self.debug:
                self.gcode.respond_info(
                    f"Set default run_current: {current_min} mA to run_current_min"
                )
        else:
            current_min = int(current_min)

        if current_max is None:
            current_max = run_current
            if self.debug:
                self.gcode.respond_info(
                    f"Set default run_current: {current_max} mA to run_current_max"
                )
        else:
            current_max = int(current_max)

        if measurement_mode == MeasurementMode.Resonances:
            current_max = current_min

        return current_min, current_max

    def get_default_stepper_parameters(
        self,
        steppers: list[str],
    ) -> tuple[int, int, int, int, int, int, int, int]:
        """Return default stepper parameters.

        Args:
            steppers (list[str]): The main and secondary stepper.

        Returns:
            tuple[int, int, int, int, int, int, int, int]: The default stepper
                parameters.
        """
        tbl_max = tbl_min = self.driver_settings[steppers[0]].get("driver_tbl")
        toff_max = toff_min = self.driver_settings[steppers[0]].get("driver_toff")
        hstrt_max = hstrt_min = self.driver_settings[steppers[0]].get("driver_hstrt")
        hend_max = hend_min = self.driver_settings[steppers[0]].get("driver_hend")

        return (
            tbl_min,
            tbl_max,
            toff_min,
            toff_max,
            hstrt_min,
            hstrt_max,
            hend_min,
            hend_max,
        )

    def configure_speed_limits(
        self,
        min_speed: None | float,
        max_speed: None | float,
        speed_change_step: None | float,
        measure_time: float,
        axes: list[str],
        steppers: list[str],
        a_axis_min: float,
        a_axis_max: float,
        acceleration: float,
    ) -> tuple[float, float, float]:
        """Configure speed limits.

        Args:
            min_speed (float | str): The in speed value, or can be set to None
                to auto calculate the value over the required RPM value.
            max_speed (float | str): The max speed value, or can be set to None
                to auto calculate the value over the required RPM value.
            speed_change_step (float | str): The step in each iteration the
                speed will be increased to.
            measure_time (float): The measurement time in seconds.
            axes (list[str]): The main and secondary axis.
            steppers (list[str]): The main and secondary stepper.
            a_axis_min (float): The minimum position of the main axis.
            a_axis_max (float): The maximum position of the main axis.
            acceleration (float): The acceleration for the movement.

        Returns:
            tuple[float, float, float]: The minimum speed, maximum speed and
                speed change step.
        """
        # In vibration measurement mode,
        # search and take registers from printer.cfg,
        # to set the speed range
        if self.measurement_mode == MeasurementMode.Resonances:
            rotation_dist = self.stepper_settings[steppers[0]].get("rotation_distance")
            # get gear ratio
            gear_ratio = self.stepper_settings[steppers[0]].get("gear_ratio")
            if not gear_ratio:  # can be () or None
                gear_ratio = ((1, 1),)
            gear_ratio = tuple(float(r) for r in gear_ratio[0])
            full_steps_per_rotation = self.stepper_settings[steppers[0]].get(
                "full_steps_per_rotation", 200
            )

            steps_multiplier = (
                full_steps_per_rotation
                / 200
                / (float(gear_ratio[0]) / float(gear_ratio[1]))
                * rotation_dist
                / 60  # to convert to mm/s from mm/min
            )

            if min_speed is None:
                min_speed = float(self.required_rpm[0] * steps_multiplier)
            else:
                min_speed = float(min_speed)

            if max_speed is None:
                max_required_speed = float(self.required_rpm[1] * steps_multiplier)
                max_speed = min(
                    (
                        (
                            -acceleration * measure_time
                            + (
                                (acceleration * measure_time) ** 2
                                + 4 * acceleration * (a_axis_max - a_axis_min)
                            )
                            ** 0.5
                        )
                        / 2
                    ),
                    max_required_speed,
                )
            else:
                max_speed = float(max_speed)

            if speed_change_step is None:
                speed_change_step = self.required_rpm[2] * steps_multiplier
            else:
                speed_change_step = float(speed_change_step)
        else:
            # Protect not defined speed & converting str -> float
            if min_speed is None or max_speed is None:
                raise self.printer.command_error(
                    "WARNING!!! Resonance speed must be defined"
                )
            min_speed, max_speed = float(min_speed), float(max_speed)
            speed_change_step = 1

        # Check speed limit
        if axes[0] == "z":
            max_velocity = self.settings["printer"].get("max_z_velocity", 0)
        else:
            max_velocity = self.settings["printer"].get("max_velocity")

        if max_speed > max_velocity:
            raise self.printer.command_error(
                f"WARNING!!! Required speed ({max_speed} mm/s) on axis ({axes[0]}) "
                f"is faster than kinematics allow ({max_velocity}), "
                f"please lower speed or increase speed limit in printer.cfg"
            )

        return min_speed, max_speed, speed_change_step

    def calculate_travel_distance(
        self,
        axes: list[str],
        a_axis_min: float,
        a_axis_max: float,
        max_speed: float,
        acceleration: float,
        measure_time: float,
        travel_distance: float | str,
    ) -> float:
        """Calculate travel distance.

        Args:
            axes (list[str]): The main and secondary axis.
            a_axis_min (float): The minimum position of the main axis.
            a_axis_max (float): The maximum position of the main axis.
            max_speed (float): The maximum speed of the main axis.
            acceleration (float): The acceleration of the main axis.
            measure_time (float): The measurement time in seconds.
            travel_distance (None | float): The travel distance, or can be set
                to None to calculate the travel distance with the
                `measure_time`, `max_speed` and `accel_decel_distance`.

        Returns:
            float: The calculated travel distance.
        """
        # Calculate min required toolhead travel distance from speed,
        # acceleration and time
        accel_decel_distance = max_speed**2 / acceleration
        auto_travel_distance = accel_decel_distance + (max_speed * measure_time)
        if self.debug:
            self.gcode.respond_info(
                f"Acceleration & deceleration zone = {accel_decel_distance} mm"
            )
            self.gcode.respond_info(
                "Auto calculated min required travel distance = "
                f"{auto_travel_distance} mm"
            )

        # Protect exceeding axis limits & calculate travel distance
        if travel_distance is None:
            if a_axis_min + auto_travel_distance > a_axis_max:
                raise self.printer.command_error(
                    f"WARNING!!! Required travel distance on axis ({axes[0]}) "
                    f"({auto_travel_distance:.1f} mm) is longer than kinematics "
                    "allows, please lower speed or increase acceleration"
                )

            travel_distance = auto_travel_distance
        else:
            travel_distance = int(travel_distance)
            if a_axis_min + travel_distance > a_axis_max:
                travel_distance = a_axis_max - a_axis_min
                if travel_distance < auto_travel_distance:
                    raise self.printer.command_error(
                        f"WARNING!!! Travel distance on axis ({axes[0]}) is "
                        "less than it should be, please increase acceleration "
                        "or lower speed"
                    )
                if travel_distance > auto_travel_distance:
                    self.gcode.respond_info(
                        f"WARNING!!! Travel distance on axis ({axes[0]}) "
                        "is longer than kinematics allows, lowering..."
                    )
            elif travel_distance < auto_travel_distance:
                travel_distance = auto_travel_distance
                if a_axis_min + auto_travel_distance > a_axis_max:
                    raise self.printer.command_error(
                        f"WARNING!!! Travel distance on axis ({axes[0]}) "
                        f"is less than required ({auto_travel_distance:.1f} mm), "
                        "and longer than kinematics allows, please lower "
                        "speed or increase acceleration"
                    )

        return travel_distance

    def display_process_summary(
        self,
        current_min: int,
        current_max: int,
        tbl_min: int,
        tbl_max: int,
        toff_min: int,
        toff_max: int,
        hstrt_min: int,
        hstrt_max: int,
        hend_min: int,
        hend_max: int,
        tpfd_min: int,
        tpfd_max: int,
        min_speed: float,
        max_speed: float,
        speed_change_step: float,
        iterations: int,
        search_method: SearchMethod,
        a_axis_min: float,
        travel_distance: float,
    ) -> None:
        """Display process information.

        Args:
            current_min (int): The minimum current value.
            current_max (int): The maximum current value.
            tbl_min (int): The minimum TBL value.
            tbl_max (int): The maximum TBL value.
            toff_min (int): The minimum TOFF value.
            toff_max (int): The maximum TOFF value.
            hstrt_min (int): The minimum HSTRT value.
            hstrt_max (int): The maximum HSTRT value.
            hend_min (int): The minimum HEND value.
            hend_max (int): The maximum HEND value.
            tpfd_min (int): The minimum TPFD value.
            tpfd_max (int): The maximum TPFD value.
            min_speed (float): The minimum speed value.
            max_speed (float): The maximum speed value.
            speed_change_step (float): The speed change step value.
            iterations (int): The number of iterations.
            search_method (SearchMethod): The search method.
            a_axis_min (float): The minimum position of the main axis.
            travel_distance (float): The travel distance value.
        """
        if self.measurement_mode == MeasurementMode.Resonances:
            # Resonance measurement mode uses the minimum values for registers.
            self.gcode.respond_info(
                f"Final max travel distance = {travel_distance:.1f} mm, "
                f"position min = {a_axis_min:.1f}, "
                f"traveling = {a_axis_min:.1f} --> {travel_distance + a_axis_min:.1f}"
            )
            self.gcode.respond_info(
                f"Start find resonances mode\n"
                f"Method     : {search_method}\n"
                f"speed      : {min_speed:.2f} --> {max_speed:.2f} mm/s with "
                f"{speed_change_step:.2f} mm/s step\n"
                f"current    : {current_min} mA\n"
                f"TBL        : {tbl_min}\n"
                f"TOFF       : {toff_min}\n"
                f"HSTRT      : {hstrt_min}\n"
                f"HEND       : {hend_min}"
            )
        else:
            self.gcode.respond_info(
                f"Final travel distance = {travel_distance:.1f} mm, "
                f"position min = {a_axis_min:.1f}, "
                f"traveling = {a_axis_min:.1f} --> {travel_distance + a_axis_min:.1f}"
            )
            self.gcode.respond_info(
                "Start of register enumeration mode\n"
                f"Method     : {search_method}\n"
                f"speed      : {min_speed:.2f} --> {max_speed:.2f} mm/s\n"
                f"current    : {current_min} --> {current_max} mA\n"
                f"iterations : {iterations}\n"
                f"TBL        : {tbl_min} --> {tbl_max}\n"
                f"TOFF       : {toff_min} --> {toff_max}\n"
                f"HSTRT      : {hstrt_min} --> {hstrt_max}\n"
                f"HEND       : {hend_min} --> {hend_max}\n"
                f"TPFD       : {tpfd_min} --> {tpfd_max}"
            )

    def home(self) -> None:
        """Home."""
        # U1 policy: home X/Y only (no Z) for chopper tuning.
        #
        # Note: Snapmaker's `homing_xyz_override` may perform a Z-hop when Z is
        # not homed. Avoid re-homing if XY are already homed.
        curtime = self.reactor.monotonic()
        status = self.toolhead.get_status(curtime)
        homed_axes = set(str(status.get("homed_axes", "")).lower())
        if "x" in homed_axes and "y" in homed_axes:
            return
        self.gcode.run_script_from_command("G28 X Y")
        self.toolhead.wait_moves()

    def get_standing_acceleration(self) -> list[Accel_Measurement]:
        """Get standing accelerometer samples.

        This will help removing the effect of gravity + static vibrations from
        real vibration data.

        Returns:
            list[Accel_Measurement]: The measurement data samples.
        """
        self.toolhead.wait_moves()
        accel_chip_name = self.get_accelerometer_chip("default")
        accel_obj = self.printer.lookup_object(accel_chip_name)
        with AccelerometerMeasure(accel_obj) as accelerometer_measurement:
            self.toolhead.dwell(5.0)
        return accelerometer_measurement.samples

    def measure_vibrations(
        self,
        coord_generator: CoordGenerator,
        travel_distance: float,
        speed: float,
    ) -> list[Accel_Measurement]:
        """Perform vibration measurement.

        Args:
            coord_generator (CoordGenerator): The coordinate generator.
            travel_distance (float): The travel distance.
            speed (float): The speed for the measurement.
            accel_chip (str): Accelerometer chip name, i.e adxl345.
            name (str): The name of the measurement.

        Returns:
            list[Accel_Measurement]: The measurement data samples.
        """
        # Start accel_chip data collection
        accel_chip_name = self.get_accelerometer_chip("default")
        accel_obj = self.printer.lookup_object(accel_chip_name)
        with AccelerometerMeasure(accel_obj) as accelerometer_measurement:
            next_coord = coord_generator.next(travel_distance)
            self.toolhead.manual_move(next_coord, speed)
        # Move to the initial position
        self.toolhead.manual_move(self.initial_position, self.travel_speed)
        coord_generator.current_coord.x = self.initial_position.x
        coord_generator.current_coord.y = self.initial_position.y
        coord_generator.current_coord.z = self.initial_position.z
        self.toolhead.wait_moves()
        self.number_of_real_samples += 1
        return accelerometer_measurement.samples

    def calculate_frequency(self, tbl: int, toff: int) -> float:
        """Calculate frequency based on TBL and TOFF values.

        Args:
            tbl (int): The TBL value.
            toff (int): The TOFF value.

        Returns:
            float: The calculated frequency in Hz.
        """
        return 1 / (
            2 * (12 + 32 * toff) * 1 / (1000000 * self.fclk)
            + 2 * 1 / (1000000 * self.fclk) * 16 * (1.5**tbl)
        )

    def get_initial_direction(self, axes: list[str]) -> Coord:
        """Return the initial direction based on the axes and kinematics.

        Args:
            axes (list[str]): The main and secondary axis.

        Returns:
            Coord: The initial direction.
        """
        initial_direction = Coord((1, 0, 0))
        if axes[0] == "x":
            if self.kinematics == "corexy":
                initial_direction = Coord((1, 1, 0)).unitize()
            else:
                initial_direction = Coord((1, 0, 0))
        elif axes[0] == "y":
            if self.kinematics == "corexy":
                initial_direction = Coord((-1, 1, 0)).unitize()
            else:
                initial_direction = Coord((0, 1, 0))
        elif axes[0] == "z":
            initial_direction = Coord((0, 0, 1))
        return initial_direction

    def wait_for_file_write(self, data_path: str) -> None:
        """Wait for file write to complete.

        Args:
            data_path (str): The path to the CSV file.
        """
        start_time = self.reactor.monotonic()
        max_wait_time = 10  # seconds
        prev_size = -1
        curr_size = 0

        while True:
            curr_size = os.path.getsize(data_path) if os.path.exists(data_path) else 0
            if curr_size > 0 and curr_size == prev_size:
                break

            prev_size = curr_size

            # Yield control back to Klipper for 100ms
            self.reactor.pause(self.reactor.monotonic() + 0.1)

            if self.reactor.monotonic() - start_time > max_wait_time:
                self.gcode.respond_info(f"Timeout waiting for file: {data_path}")
                break

    def calc_static_magnitude(
        self, samples: list[Accel_Measurement]
    ) -> tuple[float, float, float]:
        """Calculate static acceleration data from CSV file.

        Args:
            samples (list[Accel_Measurement]): The list of acceleration data samples.

        Returns:
            tuple[float, float, float]: Mean static acceleration vector.
        """
        if not samples:
            return (0.0, 0.0, 0.0)

        # Mean baseline vector (gravity + static vibration).
        sx = sy = sz = 0.0
        for s in samples:
            sx += float(getattr(s, "accel_x"))
            sy += float(getattr(s, "accel_y"))
            sz += float(getattr(s, "accel_z"))
        n = float(len(samples))
        return (sx / n, sy / n, sz / n)

    def _single_pole_lpf(
        self, values: list[float], cutoff_hz: float, sample_rate_hz: float
    ) -> list[float]:
        """Cheap IIR low-pass filter (pure Python).

        y[n] = y[n-1] + alpha * (x[n] - y[n-1])
        """
        if not values:
            return []
        if cutoff_hz <= 0 or sample_rate_hz <= 0:
            return list(values)
        dt = 1.0 / float(sample_rate_hz)
        rc = 1.0 / (2.0 * math.pi * float(cutoff_hz))
        alpha = dt / (rc + dt)

        out: list[float] = []
        y = float(values[0])
        out.append(y)
        for x in values[1:]:
            y = y + alpha * (float(x) - y)
            out.append(y)
        return out

    def _quantile(self, sorted_values: list[float], q: float) -> float:
        """Linear-interpolated quantile for a sorted list."""
        if not sorted_values:
            return 0.0
        if q <= 0:
            return float(sorted_values[0])
        if q >= 1:
            return float(sorted_values[-1])
        n = len(sorted_values)
        pos = (n - 1) * float(q)
        lo = int(math.floor(pos))
        hi = int(math.ceil(pos))
        if lo == hi:
            return float(sorted_values[lo])
        frac = pos - lo
        return float(sorted_values[lo]) * (1.0 - frac) + float(sorted_values[hi]) * frac

    def calc_magnitude(
        self, samples: list[Accel_Measurement], static_data: tuple[float, float, float]
    ) -> float:
        """Calculate median magnitude of acceleration data from CSV file.

        Args:
            samples (list[Accel_Measurement]): The list of acceleration data samples.
            static_data (tuple[float, float, float]): Mean static acceleration
                vector.

        Returns:
            float: Median magnitude of acceleration data.
        """
        if not samples:
            return float("inf")

        # Subtract baseline vector and compute magnitude.
        mags: list[float] = []
        bx, by, bz = static_data
        for s in samples:
            x = float(getattr(s, "accel_x")) - bx
            y = float(getattr(s, "accel_y")) - by
            z = float(getattr(s, "accel_z")) - bz
            mags.append(math.sqrt(x * x + y * y + z * z))

        # Determine sample rate (lis2dw on U1 exposes data_rate=1600).
        # Fallback to a sane default if not available.
        sample_rate_hz = float(getattr(self, "_accel_data_rate_hz", 1600.0))

        cutoff_hz = float(getattr(self, "_lpf_cutoff_hz", 150.0))
        mags = self._single_pole_lpf(mags, cutoff_hz=cutoff_hz, sample_rate_hz=sample_rate_hz)

        mags_sorted = sorted(mags)
        lo = self._quantile(mags_sorted, 0.20)
        hi = self._quantile(mags_sorted, 0.80)
        trimmed = [m for m in mags_sorted if lo <= m <= hi]
        if not trimmed:
            trimmed = mags_sorted
        return float(statistics.median(trimmed))

    def _u1_parse_xy_pair(self, value: object) -> tuple[float, float]:
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            return float(value[0]), float(value[1])
        if isinstance(value, str):
            parts = [p.strip() for p in value.split(",")]
            if len(parts) >= 2:
                return float(parts[0]), float(parts[1])
        raise ValueError(f"Invalid XY pair: {value!r}")

    def _u1_get_safe_xy_bounds(self, inset: float) -> tuple[float, float, float, float]:
        """Return (xmin, xmax, ymin, ymax) safe working bounds for tuning moves."""
        bed_mesh = self.settings.get("bed_mesh") if self.settings else None
        if bed_mesh and "mesh_min" in bed_mesh and "mesh_max" in bed_mesh:
            mx0, my0 = self._u1_parse_xy_pair(bed_mesh["mesh_min"])
            mx1, my1 = self._u1_parse_xy_pair(bed_mesh["mesh_max"])
            xmin, xmax = mx0, mx1
            ymin, ymax = my0, my1
        else:
            # Fallback to configured axis limits.
            xmin = float(self.stepper_settings["stepper_x"]["position_min"])
            xmax = float(self.stepper_settings["stepper_x"]["position_max"])
            ymin = float(self.stepper_settings["stepper_y"]["position_min"])
            ymax = float(self.stepper_settings["stepper_y"]["position_max"])

        xmin += inset
        ymin += inset
        xmax -= inset
        ymax -= inset
        if xmax <= xmin or ymax <= ymin:
            raise self.printer.command_error(
                f"INSET={inset} leaves no valid work area (xmin/xmax/ymin/ymax = {xmin}/{xmax}/{ymin}/{ymax})"
            )
        return xmin, xmax, ymin, ymax

    def _u1_max_travel_distance_xy(
        self,
        center_x: float,
        center_y: float,
        dir_x: float,
        dir_y: float,
        xmin: float,
        xmax: float,
        ymin: float,
        ymax: float,
    ) -> float:
        """Max travel distance centered on (center_x, center_y) along direction."""
        eps = 1e-9
        half_limits: list[float] = []
        if abs(dir_x) > eps:
            half_limits.append((center_x - xmin) / abs(dir_x))
            half_limits.append((xmax - center_x) / abs(dir_x))
        if abs(dir_y) > eps:
            half_limits.append((center_y - ymin) / abs(dir_y))
            half_limits.append((ymax - center_y) / abs(dir_y))
        if not half_limits:
            return 0.0
        half = max(0.0, min(half_limits))
        return 2.0 * half

    def _u1_set_accel(self, accel: float) -> None:
        """Set toolhead accel in a way that's compatible with vendor Klipper builds."""
        if accel is None:
            return
        accel_f = float(accel)
        if hasattr(self.toolhead, "set_accel"):
            self.toolhead.set_accel(accel_f)
            return
        # Fallback: use gcode interface if available.
        self.gcode.run_script_from_command(f"SET_VELOCITY_LIMIT ACCEL={accel_f:.3f}")

    def _u1_make_run_dirs(self, log_root: str) -> dict[str, Path]:
        base = Path(log_root)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = base / "runs" / stamp
        logs_dir = run_dir / "logs"
        results_dir = run_dir / "results"
        logs_dir.mkdir(parents=True, exist_ok=True)
        results_dir.mkdir(parents=True, exist_ok=True)
        return {"base": base, "run": run_dir, "logs": logs_dir, "results": results_dir}

    def _u1_write_json(self, path: Path, payload: object) -> None:
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
        tmp.replace(path)

    def _u1_write_samples_csv(self, path: Path, samples: list[Accel_Measurement]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w") as f:
            f.write("#time,accel_x,accel_y,accel_z\n")
            for s in samples:
                f.write(
                    "%.6f,%.6f,%.6f,%.6f\n"
                    % (
                        float(getattr(s, "time")),
                        float(getattr(s, "accel_x")),
                        float(getattr(s, "accel_y")),
                        float(getattr(s, "accel_z")),
                    )
                )

    def _u1_noise_penalty_factor(self, freq_hz: float, mode: str) -> float:
        """Return multiplicative score factor >= 1.0."""
        mode = (mode or "none").lower()
        if mode == "none":
            return 1.0
        if freq_hz <= 0:
            return 1.0
        threshold = 20_000.0
        if freq_hz >= threshold:
            return 1.0
        weight = {"moderate": 0.10, "strict": 0.25}.get(mode)
        if weight is None:
            raise self.printer.command_error(
                "NOISE_PENALTY must be one of: none, moderate, strict"
            )
        return 1.0 + weight * ((threshold / freq_hz) - 1.0)

    def _u1_measure_move(
        self,
        accel_obj: object,
        start: Coord,
        end: Coord,
        speed: float,
    ) -> list[Accel_Measurement]:
        # Move into position at travel speed (not part of measurement).
        self.toolhead.manual_move(start, self.travel_speed)
        self.toolhead.wait_moves()
        with AccelerometerMeasure(accel_obj) as m:
            self.toolhead.manual_move(end, speed)
        self.number_of_real_samples += 1
        return m.samples or []

    def _u1_lookup_accel_object(self, accel_chip_name: str) -> object:
        """Best-effort lookup for accel objects on vendor Klipper builds."""
        name = (accel_chip_name or "").strip()
        try:
            return self.printer.lookup_object(name)
        except Exception:
            pass
        # Some configs may specify only the suffix (e.g. "e0_lis2dw").
        if " " not in name:
            for prefix in ("lis2dw ", "adxl345 ", "mpu9250 "):
                try:
                    return self.printer.lookup_object(prefix + name)
                except Exception:
                    continue
        # Re-raise with a clearer message.
        raise self.printer.command_error(
            f"Unable to lookup accelerometer object '{accel_chip_name}'. "
            "Expected something like 'lis2dw e0_lis2dw'."
        )

    def _u1_measure_score(
        self,
        accel_obj: object,
        static_vec: tuple[float, float, float],
        start: Coord,
        end: Coord,
        speed: float,
        cache: dict[tuple, float],
        cache_key: tuple,
    ) -> float:
        if cache_key in cache:
            return cache[cache_key]
        total = 0.0
        for _ in range(max(1, int(self.iterations))):
            samples = self._u1_measure_move(accel_obj, start=start, end=end, speed=speed)
            total += self.calc_magnitude(samples=samples, static_data=static_vec)
            # Yield control back to Klipper.
            self.reactor.pause(self.reactor.monotonic() + 0.05)
        score = total / max(1, int(self.iterations))
        cache[cache_key] = score
        return score

    def chopper_tune(
        self,
        axis: str,
        *,
        quick: bool,
        home: bool,
        tool: int,
        accel_chip: str,
        inset: float,
        baseline_dwell: float,
        lpf_cutoff_hz: float,
        noise_penalty: str,
        raw: bool,
        log_path: str,
    ) -> dict:
        """Snapmaker U1 phased tuner (stdlib-only).

        Returns the chosen parameter set (also written via configfile.set()).
        """
        if self.printer.is_shutdown():
            raise self.printer.command_error(
                "Klipper is in shutdown state. Run FIRMWARE_RESTART (or reboot) and home the printer, then retry."
            )

        if axis not in ("x", "y"):
            raise self.printer.command_error("AXIS must be X or Y on CoreXY U1")
        if tool != 0:
            raise self.printer.command_error("TOOL must be 0 on Snapmaker U1")

        # Reset per-run state.
        self.reset_sample_data()
        self.reset_registers()

        run_dirs = self._u1_make_run_dirs(log_path)
        run_meta: dict[str, object] = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "log_root": log_path,
            "axis": axis,
            "quick": bool(quick),
            "home": bool(home),
            "tool": int(tool),
            "inset": float(inset),
            "baseline_dwell": float(baseline_dwell),
            "lpf_cutoff_hz": float(lpf_cutoff_hz),
            "noise_penalty": str(noise_penalty),
            "raw": bool(raw),
            "python": {
                "version": platform.python_version(),
                "implementation": platform.python_implementation(),
            },
            "klipper": {
                "software_version": self.printer.get_start_args().get("software_version"),
            },
        }

        # Phase 0: setup
        if home:
            self.home()
        # U1 convention (see firmware): T0 A0
        self.gcode.run_script_from_command("T0 A0")
        self.toolhead.wait_moves()

        driver, sense = self.detect_driver(stepper=axis)
        if driver not in ("2240", "5160"):
            raise self.printer.command_error(
                f"Only TMC2240/TMC5160 are supported in U1 mode (got TMC{driver})"
            )
        self.driver = driver
        self.sense_resistor = sense

        axes, steppers = self.get_axes_and_steppers(axis)
        self.steppers = list(steppers)
        self.registers["stepper_count"] = self.get_stepper_count(axis)

        accel_chip_name = self.get_accelerometer_chip(accel_chip if accel_chip else "default")
        accel_obj = self._u1_lookup_accel_object(accel_chip_name)
        self._accel_data_rate_hz = float(getattr(accel_obj, "data_rate", 1600.0))
        self._lpf_cutoff_hz = float(lpf_cutoff_hz)

        # Motion bounds (use bed_mesh safe box if present).
        xmin, xmax, ymin, ymax = self._u1_get_safe_xy_bounds(inset=inset)
        center_x = (xmin + xmax) / 2.0
        center_y = (ymin + ymax) / 2.0
        home_pos = Coord(self.toolhead.get_position())
        center = Coord((center_x, center_y, home_pos.z))

        # Direction and max travel distance along that direction.
        direction = self.get_initial_direction(list(axes))
        avail_dist = self._u1_max_travel_distance_xy(
            center_x=center.x,
            center_y=center.y,
            dir_x=direction.x,
            dir_y=direction.y,
            xmin=xmin,
            xmax=xmax,
            ymin=ymin,
            ymax=ymax,
        )
        if avail_dist <= 0:
            raise self.printer.command_error("No available travel distance in safe bounds")

        acceleration, self.travel_speed = self.get_travel_speed_and_acceleration(list(axes))
        self._u1_set_accel(acceleration)

        # Speed range (use required_rpm from config).
        measure_time = float(self.measure_time) / 1000.0
        rotation_dist = self.stepper_settings[steppers[0]].get("rotation_distance")
        gear_ratio = self.stepper_settings[steppers[0]].get("gear_ratio") or ((1, 1),)
        gear_ratio = tuple(float(r) for r in gear_ratio[0])
        full_steps_per_rotation = self.stepper_settings[steppers[0]].get(
            "full_steps_per_rotation", 200
        )
        steps_multiplier = (
            full_steps_per_rotation
            / 200
            / (float(gear_ratio[0]) / float(gear_ratio[1]))
            * rotation_dist
            / 60.0
        )

        min_speed = float(self.required_rpm[0] * steps_multiplier)
        max_required_speed = float(self.required_rpm[1] * steps_multiplier)
        speed_step = float(self.required_rpm[2] * steps_multiplier)

        # Limit max speed by available travel distance for the requested measure_time.
        # Solve v^2/accel + v*t <= avail_dist for v.
        max_speed_by_dist = (
            (-acceleration * measure_time)
            + math.sqrt((acceleration * measure_time) ** 2 + 4.0 * acceleration * avail_dist)
        ) / 2.0
        max_velocity = float(self.settings["printer"].get("max_velocity"))
        max_speed = min(max_required_speed, max_speed_by_dist, max_velocity)
        if max_speed < min_speed:
            self.gcode.respond_info(
                f"WARNING: computed max_speed={max_speed:.1f} < min_speed={min_speed:.1f}; clamping to min_speed"
            )
            max_speed = min_speed

        # Compute desired travel distance for max_speed; clamp to bounds if needed.
        accel_decel_distance = (max_speed * max_speed) / float(acceleration)
        desired_dist = accel_decel_distance + (max_speed * measure_time)
        travel_dist = min(desired_dist, avail_dist)
        if travel_dist < desired_dist:
            self.gcode.respond_info(
                f"WARNING: travel distance clamped by safe bounds: {travel_dist:.1f}mm < {desired_dist:.1f}mm"
            )

        start = center - direction * (travel_dist / 2.0)
        end = center + direction * (travel_dist / 2.0)

        # Baseline noise (standing still).
        self.toolhead.manual_move(center, self.travel_speed)
        self.toolhead.wait_moves()
        with AccelerometerMeasure(accel_obj) as am:
            self.toolhead.dwell(float(baseline_dwell))
        static_vec = self.calc_static_magnitude(am.samples or [])
        static_mag = math.sqrt(static_vec[0] ** 2 + static_vec[1] ** 2 + static_vec[2] ** 2)
        self.static_noise_vector = static_vec
        self.static_noise_magnitude = static_mag

        self.gcode.respond_info(
            f"U1 phased mode | axis={axis.upper()} | driver=TMC{driver} | accel={accel_chip_name}\n"
            f"Safe box: X {xmin:.1f}..{xmax:.1f}, Y {ymin:.1f}..{ymax:.1f} (INSET={inset})\n"
            f"Travel: {travel_dist:.1f}mm (available {avail_dist:.1f}mm) | Speed range: {min_speed:.1f}..{max_speed:.1f} mm/s\n"
            f"Static noise magnitude: {static_mag:.1f} mm/s²"
        )

        # Cache: (key, speed, dir) -> score
        score_cache: dict[tuple, float] = {}
        measurements: list[dict[str, object]] = []

        def measure_bidir_current(speed: float) -> tuple[float, float, float]:
            """Bidirectional score at current register state (no SET_TMC_FIELD)."""
            sp = round(float(speed), 4)
            fwd = self._u1_measure_score(
                accel_obj,
                static_vec,
                start=start,
                end=end,
                speed=sp,
                cache=score_cache,
                cache_key=(("baseline",), sp, "fwd"),
            )
            rev = self._u1_measure_score(
                accel_obj,
                static_vec,
                start=end,
                end=start,
                speed=sp,
                cache=score_cache,
                cache_key=(("baseline",), sp, "rev"),
            )
            return fwd, rev, max(fwd, rev)

        def measure_bidir_tbl_toff(tbl: int, toff: int, speed: float) -> tuple[float, float, float]:
            """Phase 2: vary TBL/TOFF only; keep other registers at their current values."""
            params_key = ("tbl_toff", int(tbl), int(toff))
            sp = round(float(speed), 4)
            self.apply_registers(self.steppers, "tbl", int(tbl))
            self.apply_registers(self.steppers, "toff", int(toff))
            fwd = self._u1_measure_score(
                accel_obj,
                static_vec,
                start=start,
                end=end,
                speed=sp,
                cache=score_cache,
                cache_key=(params_key, sp, "fwd"),
            )
            rev = self._u1_measure_score(
                accel_obj,
                static_vec,
                start=end,
                end=start,
                speed=sp,
                cache=score_cache,
                cache_key=(params_key, sp, "rev"),
            )
            return fwd, rev, max(fwd, rev)

        def measure_bidir_hyst(tbl: int, toff: int, hstrt: int, hend: int, speed: float) -> tuple[float, float, float]:
            """Phase 3: vary HSTRT/HEND for a selected (TBL,TOFF)."""
            params_key = ("hyst", int(tbl), int(toff), int(hstrt), int(hend))
            sp = round(float(speed), 4)
            self.apply_registers(self.steppers, "tbl", int(tbl))
            self.apply_registers(self.steppers, "toff", int(toff))
            self.apply_registers(self.steppers, "hstrt", int(hstrt))
            self.apply_registers(self.steppers, "hend", int(hend))
            fwd = self._u1_measure_score(
                accel_obj,
                static_vec,
                start=start,
                end=end,
                speed=sp,
                cache=score_cache,
                cache_key=(params_key, sp, "fwd"),
            )
            rev = self._u1_measure_score(
                accel_obj,
                static_vec,
                start=end,
                end=start,
                speed=sp,
                cache=score_cache,
                cache_key=(params_key, sp, "rev"),
            )
            return fwd, rev, max(fwd, rev)

        def measure_bidir_tpfd(
            tbl: int, toff: int, hstrt: int, hend: int, tpfd: int, speed: float
        ) -> tuple[float, float, float]:
            """Phase 4: vary TPFD for a selected (TBL,TOFF,HSTRT,HEND)."""
            params_key = ("tpfd", int(tbl), int(toff), int(hstrt), int(hend), int(tpfd))
            sp = round(float(speed), 4)
            self.apply_registers(self.steppers, "tbl", int(tbl))
            self.apply_registers(self.steppers, "toff", int(toff))
            self.apply_registers(self.steppers, "hstrt", int(hstrt))
            self.apply_registers(self.steppers, "hend", int(hend))
            self.apply_registers(self.steppers, "tpfd", int(tpfd))
            fwd = self._u1_measure_score(
                accel_obj,
                static_vec,
                start=start,
                end=end,
                speed=sp,
                cache=score_cache,
                cache_key=(params_key, sp, "fwd"),
            )
            rev = self._u1_measure_score(
                accel_obj,
                static_vec,
                start=end,
                end=start,
                speed=sp,
                cache=score_cache,
                cache_key=(params_key, sp, "rev"),
            )
            return fwd, rev, max(fwd, rev)

        def pick_speeds() -> tuple[float, float, float, list[dict[str, object]]]:
            # Phase 1: adaptive resonance speed selection (limited sampling).
            n0 = 9 if quick else 13
            speeds = [min_speed + i * (max_speed - min_speed) / (n0 - 1) for i in range(n0)]

            scored: list[tuple[float, float]] = []
            speed_records: list[dict[str, object]] = []
            for sp in speeds:
                _, _, s = measure_bidir_current(sp)
                scored.append((float(sp), float(s)))
                speed_records.append({"speed": float(sp), "score": float(s)})

            # refine around best speed
            best_sp, best_score = max(scored, key=lambda t: t[1])
            window = (max_speed - min_speed) / max(4.0, float(n0))
            for _ in range(2 if quick else 3):
                cand = [best_sp - window, best_sp - window / 2, best_sp, best_sp + window / 2, best_sp + window]
                cand = [sp for sp in cand if min_speed <= sp <= max_speed]
                for sp in cand:
                    _, _, s = measure_bidir_current(sp)
                    if s > best_score:
                        best_sp, best_score = float(sp), float(s)
                window *= 0.5

            speed_peak = float(best_sp)
            # mid: 2nd best away from the peak, else midpoint.
            scored_sorted = sorted(scored, key=lambda t: t[1], reverse=True)
            min_sep = 0.10 * (max_speed - min_speed)
            speed_mid = None
            for sp, sc in scored_sorted[1:]:
                if abs(sp - speed_peak) >= min_sep:
                    speed_mid = float(sp)
                    break
            if speed_mid is None:
                speed_mid = float((min_speed + max_speed) / 2.0)
            speed_high = float(min_speed + 0.80 * (max_speed - min_speed))
            return speed_peak, speed_mid, speed_high, speed_records

        speed_peak, speed_mid, speed_high, speed_scan = pick_speeds()
        self.gcode.respond_info(
            f"Selected speeds: peak={speed_peak:.1f}, mid={speed_mid:.1f}, high={speed_high:.1f} mm/s"
        )

        # Phase 5 baseline validation measurements (before changing any registers).
        baseline_scores = {}
        for name, sp in (("peak", speed_peak), ("mid", speed_mid), ("high", speed_high)):
            _, _, s = measure_bidir_current(sp)
            baseline_scores[name] = float(s)

        raw_files: list[str] = []
        if raw:
            raw_dir = run_dirs["results"] / "raw"
            # Capture raw accel samples for baseline and tuned at the peak speed only
            # (full raw capture for every scan point would be too large).
            base_fwd = self._u1_measure_move(accel_obj, start=start, end=end, speed=float(speed_peak))
            base_rev = self._u1_measure_move(accel_obj, start=end, end=start, speed=float(speed_peak))
            p1 = raw_dir / "baseline_peak_fwd.csv"
            p2 = raw_dir / "baseline_peak_rev.csv"
            self._u1_write_samples_csv(p1, base_fwd)
            self._u1_write_samples_csv(p2, base_rev)
            raw_files += [str(p1), str(p2)]

        # Phase 2: coarse scan TBL x TOFF
        coarse_rows: list[dict[str, object]] = []
        for tbl in range(0, 4):
            for toff in range(1, 9):
                freq_hz = float(self.calculate_frequency(tbl, toff))
                penalty_factor = self._u1_noise_penalty_factor(freq_hz=freq_hz, mode=noise_penalty)

                speeds_to_check = [("peak", speed_peak)]
                if not quick:
                    speeds_to_check += [("mid", speed_mid), ("high", speed_high)]

                per_speed: dict[str, float] = {}
                for label, sp in speeds_to_check:
                    fwd, rev, s = measure_bidir_tbl_toff(tbl, toff, sp)
                    per_speed[label] = float(s)
                    measurements.append(
                        {
                            "phase": "coarse",
                            "tbl": tbl,
                            "toff": toff,
                            "hstrt": None,  # kept current
                            "hend": None,  # kept current
                            "tpfd": None,  # kept current
                            "speed": float(sp),
                            "fwd": float(fwd),
                            "rev": float(rev),
                            "score": float(s),
                            "freq_hz": freq_hz,
                        }
                    )

                if quick:
                    score_total = per_speed["peak"]
                else:
                    score_total = (
                        0.50 * per_speed["peak"]
                        + 0.30 * per_speed["mid"]
                        + 0.20 * per_speed["high"]
                    )
                score_total *= float(penalty_factor)

                coarse_rows.append(
                    {
                        "tbl": tbl,
                        "toff": toff,
                        "freq_hz": freq_hz,
                        "penalty_factor": float(penalty_factor),
                        "score_total": float(score_total),
                        "scores": per_speed,
                    }
                )

        coarse_rows_sorted = sorted(coarse_rows, key=lambda r: r["score_total"])
        top_k = 1 if quick else 3
        top_candidates = coarse_rows_sorted[:top_k]

        self.gcode.respond_info(
            "Top coarse candidates: "
            + ", ".join(
                [
                    f"TBL={c['tbl']} TOFF={c['toff']} score={c['score_total']:.1f}"
                    for c in top_candidates
                ]
            )
        )

        # Phase 3: fine scan HSTRT x HEND on top candidates
        fine_rows: list[dict[str, object]] = []
        for cand in top_candidates:
            tbl = int(cand["tbl"])
            toff = int(cand["toff"])
            for hstrt in range(0, 8):
                for hend in range(2, 16):
                    if hstrt + hend > 16:
                        continue
                    fwd, rev, s = measure_bidir_hyst(tbl, toff, hstrt, hend, speed_peak)
                    row = {
                        "tbl": tbl,
                        "toff": toff,
                        "hstrt": hstrt,
                        "hend": hend,
                        "tpfd": None,  # kept current
                        "speed": float(speed_peak),
                        "fwd": float(fwd),
                        "rev": float(rev),
                        "score": float(s),
                    }
                    fine_rows.append(row)
                    measurements.append({"phase": "fine", **row})

        if not fine_rows:
            raise self.printer.command_error("Fine scan produced no candidates")

        # Robust fine selection:
        # - Scan HSTRT/HEND at peak (fast, high-SNR).
        # - Then re-score only the top N at additional speeds to avoid choosing a
        #   candidate that only improves at one speed.
        fine_rows_sorted = sorted(fine_rows, key=lambda r: float(r["score"]))
        fine_top_n = 3 if quick else 5
        fine_shortlist = fine_rows_sorted[: max(1, min(fine_top_n, len(fine_rows_sorted)))]

        fine_ranked: list[dict[str, object]] = []
        for r in fine_shortlist:
            tbl = int(r["tbl"])
            toff = int(r["toff"])
            hstrt = int(r["hstrt"])
            hend = int(r["hend"])

            per_speed = {"peak": float(r["score"])}
            extra_speeds = [("high", speed_high)] if quick else [("mid", speed_mid), ("high", speed_high)]
            for label, sp in extra_speeds:
                fwd, rev, s = measure_bidir_hyst(tbl, toff, hstrt, hend, sp)
                per_speed[label] = float(s)
                measurements.append(
                    {
                        "phase": "fine_rescore",
                        "label": label,
                        "tbl": tbl,
                        "toff": toff,
                        "hstrt": hstrt,
                        "hend": hend,
                        "tpfd": None,
                        "speed": float(sp),
                        "fwd": float(fwd),
                        "rev": float(rev),
                        "score": float(s),
                    }
                )

            if quick:
                score_total = 0.70 * per_speed["peak"] + 0.30 * per_speed["high"]
            else:
                score_total = (
                    0.50 * per_speed["peak"]
                    + 0.30 * per_speed["mid"]
                    + 0.20 * per_speed["high"]
                )

            freq_hz = float(self.calculate_frequency(tbl, toff))
            score_total *= float(self._u1_noise_penalty_factor(freq_hz=freq_hz, mode=noise_penalty))

            fine_ranked.append({**r, "scores": per_speed, "score_total": float(score_total)})

        fine_ranked_sorted = sorted(fine_ranked, key=lambda rr: float(rr["score_total"]))
        fine_best = fine_ranked_sorted[0]
        self.gcode.respond_info(
            "Top fine candidates: "
            + ", ".join(
                [
                    f"HSTRT={int(c['hstrt'])} HEND={int(c['hend'])} score={float(c['score_total']):.1f}"
                    for c in fine_ranked_sorted[: min(3, len(fine_ranked_sorted))]
                ]
            )
        )

        # Phase 4: TPFD scan (2240/5160)
        tuned = dict(fine_best)
        tpfd_rows: list[dict[str, object]] = []
        if driver in ("2240", "5160"):
            for tpfd in range(0, 16):
                fwd, rev, s = measure_bidir_tpfd(
                    tuned["tbl"],
                    tuned["toff"],
                    tuned["hstrt"],
                    tuned["hend"],
                    tpfd,
                    speed_peak,
                )
                row = {
                    "tbl": tuned["tbl"],
                    "toff": tuned["toff"],
                    "hstrt": tuned["hstrt"],
                    "hend": tuned["hend"],
                    "tpfd": tpfd,
                    "speed": float(speed_peak),
                    "fwd": float(fwd),
                    "rev": float(rev),
                    "score": float(s),
                }
                tpfd_rows.append(row)
                measurements.append({"phase": "tpfd", **row})

            # Robust TPFD selection: re-score top N at extra speeds.
            tpfd_rows_sorted = sorted(tpfd_rows, key=lambda r: float(r["score"]))
            tpfd_top_n = 3 if quick else 5
            tpfd_shortlist = tpfd_rows_sorted[: max(1, min(tpfd_top_n, len(tpfd_rows_sorted)))]

            tpfd_ranked: list[dict[str, object]] = []
            for r in tpfd_shortlist:
                tbl = int(r["tbl"])
                toff = int(r["toff"])
                hstrt = int(r["hstrt"])
                hend = int(r["hend"])
                tpfd = int(r["tpfd"])

                per_speed = {"peak": float(r["score"])}
                extra_speeds = [("high", speed_high)] if quick else [("mid", speed_mid), ("high", speed_high)]
                for label, sp in extra_speeds:
                    fwd, rev, s = measure_bidir_tpfd(tbl, toff, hstrt, hend, tpfd, sp)
                    per_speed[label] = float(s)
                    measurements.append(
                        {
                            "phase": "tpfd_rescore",
                            "label": label,
                            "tbl": tbl,
                            "toff": toff,
                            "hstrt": hstrt,
                            "hend": hend,
                            "tpfd": tpfd,
                            "speed": float(sp),
                            "fwd": float(fwd),
                            "rev": float(rev),
                            "score": float(s),
                        }
                    )

                if quick:
                    score_total = 0.70 * per_speed["peak"] + 0.30 * per_speed["high"]
                else:
                    score_total = (
                        0.50 * per_speed["peak"]
                        + 0.30 * per_speed["mid"]
                        + 0.20 * per_speed["high"]
                    )

                freq_hz = float(self.calculate_frequency(tbl, toff))
                score_total *= float(self._u1_noise_penalty_factor(freq_hz=freq_hz, mode=noise_penalty))

                tpfd_ranked.append({**r, "scores": per_speed, "score_total": float(score_total)})

            tpfd_ranked_sorted = sorted(tpfd_ranked, key=lambda rr: float(rr["score_total"]))
            tuned = dict(tpfd_ranked_sorted[0])
            self.gcode.respond_info(
                "Top TPFD candidates: "
                + ", ".join(
                    [
                        f"TPFD={int(c['tpfd'])} score={float(c['score_total']):.1f}"
                        for c in tpfd_ranked_sorted[: min(3, len(tpfd_ranked_sorted))]
                    ]
                )
            )

        # Apply tuned registers (final).
        self.apply_registers(self.steppers, "tbl", int(tuned["tbl"]))
        self.apply_registers(self.steppers, "toff", int(tuned["toff"]))
        self.apply_registers(self.steppers, "hstrt", int(tuned["hstrt"]))
        self.apply_registers(self.steppers, "hend", int(tuned["hend"]))
        self.apply_registers(self.steppers, "tpfd", int(tuned.get("tpfd", 0)))

        if raw:
            raw_dir = run_dirs["results"] / "raw"
            tuned_fwd = self._u1_measure_move(accel_obj, start=start, end=end, speed=float(speed_peak))
            tuned_rev = self._u1_measure_move(accel_obj, start=end, end=start, speed=float(speed_peak))
            p1 = raw_dir / "tuned_peak_fwd.csv"
            p2 = raw_dir / "tuned_peak_rev.csv"
            self._u1_write_samples_csv(p1, tuned_fwd)
            self._u1_write_samples_csv(p2, tuned_rev)
            raw_files += [str(p1), str(p2)]

        # Phase 5 validation: tuned vs baseline at the 3 speeds
        tuned_scores = {}
        for name, sp in (("peak", speed_peak), ("mid", speed_mid), ("high", speed_high)):
            fwd, rev, s = measure_bidir_tpfd(
                int(tuned["tbl"]),
                int(tuned["toff"]),
                int(tuned["hstrt"]),
                int(tuned["hend"]),
                int(tuned.get("tpfd", 0)),
                sp,
            )
            tuned_scores[name] = float(s)
            measurements.append(
                {
                    "phase": "validation_tuned",
                    "tbl": int(tuned["tbl"]),
                    "toff": int(tuned["toff"]),
                    "hstrt": int(tuned["hstrt"]),
                    "hend": int(tuned["hend"]),
                    "tpfd": int(tuned.get("tpfd", 0)),
                    "speed": float(sp),
                    "fwd": float(fwd),
                    "rev": float(rev),
                    "score": float(s),
                }
            )

        validation = {
            "baseline": baseline_scores,
            "tuned": tuned_scores,
            # Positive values mean improvement (lower vibrations after tuning).
            "delta": {k: float(baseline_scores[k] - tuned_scores[k]) for k in baseline_scores},
        }

        # This tuner currently calibrates chopper (SpreadCycle) fields only.
        # Do not modify run_current.
        best_parameters = {
            "tbl": int(tuned["tbl"]),
            "toff": int(tuned["toff"]),
            "hstrt": int(tuned["hstrt"]),
            "hend": int(tuned["hend"]),
            "tpfd": int(tuned.get("tpfd", 0)),
        }

        self.save_configs(best_parameters)
        tuned_freq_hz = float(self.calculate_frequency(best_parameters["tbl"], best_parameters["toff"]))
        self.gcode.respond_info(
            "Tuned parameters:\n"
            f"  driver_tbl={best_parameters['tbl']}\n"
            f"  driver_toff={best_parameters['toff']} (f_chop~{tuned_freq_hz/1000.0:.1f} kHz)\n"
            f"  driver_hstrt={best_parameters['hstrt']}\n"
            f"  driver_hend={best_parameters['hend']}\n"
            f"  driver_tpfd={best_parameters['tpfd']}"
        )
        self.gcode.respond_info(
            "Validation (baseline - tuned, + means improvement):\n"
            f"  peak: {validation['delta']['peak']:.1f} mm/s²\n"
            f"  mid : {validation['delta']['mid']:.1f} mm/s²\n"
            f"  high: {validation['delta']['high']:.1f} mm/s²"
        )

        payload = {
            "meta": run_meta,
            "dirs": {k: str(v) for k, v in run_dirs.items()},
            "driver": {"model": f"tmc{driver}", "stepper": steppers[0], "sense_resistor": sense},
            "accel": {"chip": accel_chip_name, "data_rate_hz": self._accel_data_rate_hz},
            "safe_box": {"xmin": xmin, "xmax": xmax, "ymin": ymin, "ymax": ymax},
            "motion": {"travel_distance_mm": travel_dist, "accel": acceleration, "travel_speed": self.travel_speed},
            "speed_range": {"min": min_speed, "max": max_speed, "step": speed_step},
            "speeds": {"peak": speed_peak, "mid": speed_mid, "high": speed_high, "scan": speed_scan},
            "baseline": {"static_vector": static_vec, "static_magnitude": static_mag, "scores": baseline_scores},
            "tuned": {"params": best_parameters, "validation": validation},
            "measurements": measurements,
            "raw_files": raw_files,
        }

        self._u1_write_json(run_dirs["results"] / "run.json", payload)
        self.gcode.respond_info(
            "CHOPPER_TUNE complete. Saved JSON to "
            f"{str(run_dirs['results'] / 'run.json')}"
        )
        return best_parameters

    def generate_sample_name(
        self,
        current: int,
        tbl: int,
        toff: int,
        hstrt: int,
        hend: int,
        tpfd: int,
        speed: float,
    ) -> str:
        """Generate a sample name based on the parameters.

        Args:
            current (int): The current value.
            tbl (int): The TBL value.
            toff (int): The TOFF value.
            hstrt (int): The HSTRT value.
            hend (int): The HEND value.
            tpfd (int): The TPFD value.
            speed (float): The speed value.

        Returns:
            str: The generated sample name.
        """
        freq = self.calculate_frequency(tbl, toff)
        return (
            f"current={current}_"
            f"tbl={tbl}_"
            f"toff={toff}_"
            f"hstrt={hstrt}_"
            f"hend={hend}_"
            f"tpfd={tpfd}_"
            f"speed={speed:.2f}_"
            f"freq={freq / 1000:.2f}kHz"
        )

    # Legacy compare_results() removed in U1 mode (avoids third-party deps).

    def get_time_elapsed(self) -> float:
        """Get elapsed time since the start of the process.

        Returns:
            float: The time elapsed in seconds.
        """
        return self.reactor.monotonic() - self.global_start_time

    def get_remaining_time(self) -> float:
        """Estimate remaining time for the process.

        Returns:
            float: The estimated remaining time in seconds.
        """
        time_passed = self.get_time_elapsed()
        total_estimated_process_time = (
            time_passed / self.number_of_samples * self.total_expected_samples
        )
        return max(total_estimated_process_time - time_passed, 0)

    def convert_seconds_to_hms(self, seconds: float) -> str:
        """Convert seconds to hours, minutes, and seconds.

        Args:
            seconds (float): The time in seconds.

        Returns:
            str: The time in "HHh MMm SSs" format.
        """
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        secs = int(seconds % 60)

        # do not include hours or minutes if zero
        hours_str = f"{hours}h" if hours > 0 else ""
        minutes_str = f"{minutes}m" if minutes > 0 or hours > 0 else ""

        return f"{hours_str}{minutes_str}{secs}s"

    def progress_report(
        self, measured_vibrations: float = 0.0, iteration: int = 0, cached: bool = False
    ) -> None:
        """Report progress.

        Args:
            measured_vibrations (float): The measured vibrations.
            iteration (int): The current iteration. Only report progress on the
                first iteration.
            cached (bool): Whether the vibrations value is from a cached sample.
        """
        if iteration == 0:
            if self.total_expected_samples > 0:
                percent_complete = (
                    self.number_of_samples / self.total_expected_samples
                ) * 100
                self.gcode.respond_info(
                    f"Sample {'(cached)' if cached else '        '}: "
                    f"{self.number_of_samples}/"
                    f"{self.total_expected_samples} "
                    f"({percent_complete:0.1f}%) | "
                    f"E: {self.convert_seconds_to_hms(self.get_time_elapsed())} | "
                    f"R: {self.convert_seconds_to_hms(self.get_remaining_time())}"
                )
            else:
                self.gcode.respond_info(
                    f"Sample         : {self.number_of_samples} | "
                    f"E: {self.convert_seconds_to_hms(self.get_time_elapsed())}"
                )
        self.gcode.respond_info(
            f"Iteration {iteration + 1:<5d}: {measured_vibrations:0.1f} mm/s²"
        )

    def objective_function(self, params: list[float]) -> float:
        """Objective function for optimization.

        Args:
            params (list[float]): The parameters to optimize.

        Returns:
            float: The average measured vibrations.
        """
        self.gcode.respond_info("-------------------------------")
        current, tbl, toff, hstrt, hend, tpfd, speed = [round(p) for p in params]

        # convert speed back to the correct range
        speed = float(speed) / 100

        # Dump TMC settings
        self.gcode.respond_info(
            f"tbl={tbl} "
            f"toff={toff} "
            f"hstrt={hstrt} "
            f"hend={hend} "
            f"tpfd={tpfd} "
            f"current={current} "
            f"speed={speed:.2f}"
        )

        # penalize hstrt + hend > hstrt_hend_max
        if hstrt + hend > self.hstrt_hend_max:
            self.gcode.respond_info(
                f"Penalizing hstrt + hend > {self.hstrt_hend_max}: inf mm/s²"
            )
            self.number_of_samples += 1  # consider this as a sample
            return float("inf")

        # Set tbl, toff, hend, and hstrt values
        self.apply_registers(self.steppers, "curr", current)
        self.apply_registers(self.steppers, "tbl", tbl)
        self.apply_registers(self.steppers, "toff", toff)
        self.apply_registers(self.steppers, "hstrt", hstrt)
        self.apply_registers(self.steppers, "hend", hend)
        self.apply_registers(self.steppers, "tpfd", tpfd)

        sample_name = self.generate_sample_name(
            current=current,
            tbl=tbl,
            toff=toff,
            hstrt=hstrt,
            hend=hend,
            tpfd=tpfd,
            speed=speed,
        )
        if sample_name in self.samples:
            cached_vibrations = self.samples[sample_name]
            self.number_of_samples += 1  # consider this as a sample
            self.progress_report(cached_vibrations, cached=True)
            return cached_vibrations

        total_vibrations = 0
        self.number_of_samples += 1
        travel_distance = self.travel_distance
        if self.measurement_mode == MeasurementMode.Resonances:
            travel_distance = self.travel_distance * (speed / self.max_speed)

        for iteration in range(self.iterations):
            samples = self.measure_vibrations(
                self.coord_generator,
                travel_distance,
                speed,
            )
            measured_vibrations = self.calc_magnitude(
                samples=samples,
                static_data=self.static_noise_vector,
            )

            total_vibrations += measured_vibrations
            self.progress_report(measured_vibrations, iteration)

            # Allow other Klipper tasks to run between iterations
            self.reactor.pause(self.reactor.monotonic() + 0.05)

        total_vibrations /= self.iterations
        if self.iterations > 1:
            self.gcode.respond_info(f"Mean vibrations: {total_vibrations:0.1f} mm/s²")
        # store best result for the last report
        self.best_result = min(self.best_result, total_vibrations)

        self.samples[sample_name] = total_vibrations
        self.speed_vs_vibrations.append((speed, total_vibrations))

        # Allow other Klipper tasks to run between iterations
        self.reactor.pause(self.reactor.monotonic() + 0.05)

        return total_vibrations

    def progressive_search_loop(
        self,
        param_min: int,
        param_max: int,
        param_index: int,
        best_params: list[int],
    ) -> list[int]:
        """Progressive search loop for a single parameter.

        Args:
            param_min (int): The minimum value of the parameter.
            param_max (int): The maximum value of the parameter.
            param_index (int): The index of the parameter in the best_params list.
            best_params (list[int]): The current best parameters.
        """
        best_mag = float("inf")
        for param in range(param_min, param_max + 1):
            best_params[param_index] = param
            mag = self.objective_function(best_params)
            if mag < best_mag:
                best_mag = mag
                best_param = param
        best_params[param_index] = best_param
        return best_params

    def perform_brute_force_search(self) -> list[int]:
        """Legacy SciPy-based optimization (not supported on U1)."""
        raise self.printer.command_error(
            "SEARCH_METHOD=brute_force is not supported in this fork (no SciPy/Numpy on U1)"
        )

    def perform_adaptive_search(self) -> list[int]:
        """Legacy SciPy-based optimization (not supported on U1)."""
        raise self.printer.command_error(
            "SEARCH_METHOD=adaptive is not supported in this fork (no SciPy/Numpy on U1)"
        )

    def perform_progressive_search(self) -> list[int]:
        """Perform progressive search for optimal parameters.

        Returns:
            list[int]: The best parameters found.
        """
        self.gcode.respond_info(
            "Starting Progressive (Trinamic Flowchart) Optimization..."
        )

        # Initial "Safe" Defaults as per Datasheet
        best_current = self.current_min
        best_toff = self.toff_min if self.toff_min > 0 else 3
        best_tbl = 2
        best_hend = self.hend_min
        best_hstrt = self.hstrt_min
        best_tpfd = self.tpfd_min
        best_params = [
            best_current,
            best_tbl,
            best_toff,
            best_hstrt,
            best_hend,
            best_tpfd,
            self.min_speed * 100,
        ]
        self.total_expected_samples = (
            (self.toff_max - self.toff_min + 1)
            + (self.tbl_max - self.tbl_min + 1)
            + (self.hend_max - self.hend_min + 1)
            + (self.hstrt_max - self.hstrt_min + 1)
        )

        # Step 1: Find optimal TOFF (Base Frequency)
        # The datasheet suggests TOFF should be chosen for 20-50kHz.
        # We find the one that produces the least vibration at default hysteresis.
        self.gcode.respond_info("Step 1: Optimizing TOFF...")
        best_params = self.progressive_search_loop(
            param_min=self.toff_min,
            param_max=self.toff_max,
            param_index=2,
            best_params=best_params,
        )
        self.gcode.respond_info(f"-> Best TOFF found: {best_params[2]}")
        # Step 2: Find optimal TBL (Blank Time)
        # Clean up the comparator signal before fine-tuning hysteresis.
        self.gcode.respond_info("Step 2: Optimizing TBL...")
        best_params = self.progressive_search_loop(
            param_min=self.tbl_min,
            param_max=self.tbl_max,
            param_index=1,
            best_params=best_params,
        )
        self.gcode.respond_info(f"-> Best TBL found: {best_params[1]}")
        # Step 3: Find optimal HEND (Hysteresis End / Low-side)
        # Flowchart: Start with HSTRT=0, increase HEND until smooth.
        self.gcode.respond_info("Step 3: Optimizing HEND...")
        best_params = self.progressive_search_loop(
            param_min=self.hend_min,
            param_max=self.hend_max,
            param_index=4,
            best_params=best_params,
        )
        self.gcode.respond_info(f"-> Best HEND found: {best_params[4]}")
        # Step 4: Find optimal HSTRT (Hysteresis Start / High-side)
        # Flowchart: Use best HEND, increase HSTRT to fine-tune the zero-crossing.
        self.gcode.respond_info("Step 4: Optimizing HSTRT...")
        best_params = self.progressive_search_loop(
            param_min=self.hstrt_min,
            param_max=self.hstrt_max,
            param_index=3,
            best_params=best_params,
        )
        self.gcode.respond_info(f"-> Best HSTRT found: {best_params[3]}")

        # Finally search for TPFD if applicable
        if self.driver in ["2240", "5160"]:
            self.gcode.respond_info("Step 5: Optimizing TPFD...")
            best_params = self.progressive_search_loop(
                param_min=self.tpfd_min,
                param_max=self.tpfd_max,
                param_index=5,
                best_params=best_params,
            )
            self.gcode.respond_info(f"-> Best TPFD found: {best_params[5]}")

        return best_params

    def search_best_parameters(self) -> list[int]:
        """Run the parameter search process.

        This can use either brute-force or adaptive optimization methods depending
        on the selected search method.

        Returns:
            dict: The best parameters found.
        """
        self.gcode.respond_info("Starting optimization...")
        start_time = self.reactor.monotonic()

        if self.measurement_mode == MeasurementMode.Resonances:
            # Dump TMC settings
            self.gcode.respond_info(
                f"tbl={self.tbl_min} "
                f"toff={self.toff_min} "
                f"hstrt={self.hstrt_min} "
                f"hend={self.hend_min} "
                f"tpfd={self.tpfd_min} "
                f"current={self.current_min} "
                f"speed={self.min_speed:.2f} --> {self.max_speed:.2f}"
            )

        # set initial values
        self.apply_registers(self.steppers, "curr", self.current_min)
        self.apply_registers(self.steppers, "tbl", self.tbl_min)
        self.apply_registers(self.steppers, "toff", self.toff_min)
        self.apply_registers(self.steppers, "hend", self.hend_min)
        self.apply_registers(self.steppers, "hstrt", self.hstrt_min)
        self.apply_registers(self.steppers, "tpfd", self.tpfd_min)

        if self.search_method == SearchMethod.BruteForce:
            best_params = self.perform_brute_force_search()
        elif self.search_method == SearchMethod.Adaptive:
            best_params = self.perform_adaptive_search()
        elif self.search_method == SearchMethod.Progressive:
            best_params = self.perform_progressive_search()

        # update overall best params with final results
        overall_best_params = {
            "current": best_params[0],
            "tbl": best_params[1],
            "toff": best_params[2],
            "hstrt": best_params[3],
            "hend": best_params[4],
            "tpfd": best_params[5],
            "speed": float(best_params[6]) / 100,
        }

        duration = self.reactor.monotonic() - start_time

        result_message = (
            f"Optimization Completed in {self.convert_seconds_to_hms(duration)}\n"
            f"Number of samples : {self.number_of_samples}\n"
            f"Best Score        : {self.best_result:.1f} mm/s²\n\n"
            "Parameters\n"
            "----------\n"
            f"speed        : {overall_best_params['speed']:.2f} mm/s\n"
            f"current      : {overall_best_params['current']} mA\n"
            f"driver_tbl   : {overall_best_params['tbl']}\n"
            f"driver_toff  : {overall_best_params['toff']}\n"
            f"driver_hstrt : {overall_best_params['hstrt']}\n"
            f"driver_hend  : {overall_best_params['hend']}\n"
        )
        if self.driver in ["2240", "5160"]:
            result_message += f"driver_tpfd : {overall_best_params['tpfd']}"
        self.gcode.respond_info(result_message)

        # Apply best parameters
        self.apply_registers(self.steppers, "curr", overall_best_params["current"])
        self.apply_registers(self.steppers, "tbl", overall_best_params["tbl"])
        self.apply_registers(self.steppers, "toff", overall_best_params["toff"])
        self.apply_registers(self.steppers, "hend", overall_best_params["hend"])
        self.apply_registers(self.steppers, "hstrt", overall_best_params["hstrt"])
        self.apply_registers(self.steppers, "tpfd", overall_best_params["tpfd"])

        return overall_best_params

    def save_configs(self, best_parameters: dict | None) -> None:
        """Save the best parameters to printer.cfg.

        Args:
            best_parameters (dict | None): The best parameters found.
        """
        if best_parameters is None:
            return

        for stepper_index in range(self.registers["stepper_count"]):
            stepper_index = str(stepper_index) if stepper_index > 0 else ""
            for field, value in best_parameters.items():
                if field == "speed":
                    continue  # skip speed field

                if field == "tpfd" and self.driver not in ["2240", "5160"]:
                    continue  # skip tpfd for unsupported drivers

                field_name = f"driver_{field}" if field != "current" else "run_current"
                value = str(value) if field != "current" else f"{value / 1000:0.2f}"

                self.configfile.set(
                    f"tmc{self.driver} {self.steppers[0]}{stepper_index}",
                    field_name,
                    value,
                )

        self.gcode.respond_info(
            "Best parameters saved to printer.cfg, run SAVE_CONFIG to apply."
        )

    def plot_data(self, date_stamp: None | str = None) -> None:
        """Plot the collected vibration data.

        Args:
            date_stamp (None | str): The timestamp string to use in filenames.
        """
        import plotly.graph_objects as go
        import plotly.io as pio

        if date_stamp is None:
            date_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        params = [
            reversed(list(self.samples.items())),
            sorted(self.samples.items(), key=lambda x: x[1]),
        ]
        names = ["as_measured", "sorted"]
        plot_html_paths = []
        for param, name in zip(params, names):
            fig = go.Figure()
            for entry in param:
                toff = int(entry[0].split("_")[2].split("=")[1])
                color = COLORS[toff if toff <= 8 else toff - 8]
                fig.add_trace(
                    go.Bar(
                        x=[entry[1]],
                        y=[entry[0]],
                        marker_color=color,
                        orientation="h",
                        showlegend=False,
                    )
                )
            fig.update_layout(
                title="Median Magnitude vs Parameters",
                xaxis_title="Median Magnitude (mm/s²)",
                yaxis_title="Parameters",
                coloraxis_showscale=True,
            )
            plot_html_path = os.path.join(
                RESULTS_FOLDER,
                f"{date_stamp}"
                f"_interactive_plot_{name}"
                f"_{self.accel_chip}"
                f"_tmc{self.driver}"
                f"_{self.steppers[0]}"
                f"_{self.sense_resistor}"
                ".html",
            )
            plot_html_paths.append(plot_html_path)
            pio.write_html(fig, plot_html_path, auto_open=False)
            speed1 = params[1][0][0].split("_")[6].split("=")[1]
            speed2 = params[1][1][0].split("_")[6].split("=")[1]
            if speed1 != speed2:
                break

        # Export Info
        self.gcode.respond_info("Access to interactive plot at:")
        for plot_html_path in plot_html_paths:
            self.gcode.respond_info(f"{plot_html_path}")

    def store_data(self, date_stamp: None | str = None) -> None:
        """Store collected sample data to a JSON file.

        Args:
            date_stamp (None | str): The timestamp string to use in filenames.
        """
        if date_stamp is None:
            date_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        json_path = os.path.join(
            RESULTS_FOLDER,
            f"{date_stamp}"
            f"_sample_data"
            f"_{self.accel_chip}"
            f"_tmc{self.driver}"
            f"_{self.steppers[0]}"
            f"_{self.sense_resistor}"
            ".json",
        )
        with open(json_path, mode="w") as file:
            json.dump(self.samples, file, indent=4)
        self.gcode.respond_info(f"Sample data saved to: {json_path}")

    def cmd_chopper_tune(self, gcmd: GCodeCommand) -> bool:
        """Tune stepper values.

        Args:
            gcmd (GCodeCommand): The G-Code command.

        Returns:
            bool: True if command completed successfully, False otherwise.
        """
        self.reactor.register_callback(
            lambda e: self.parse_args_and_run_optimization(gcmd)
        )
        return True

    def parse_args_and_run_optimization(self, gcmd: GCodeCommand) -> None:
        """Collect data from G-Code command and run the U1 phased tuner."""
        try:
            axis = gcmd.get("AXIS", "x").lower()
            quick = bool(gcmd.get_int("QUICK", 0))
            home = bool(gcmd.get_int("HOME", 1))
            tool = gcmd.get_int("TOOL", 0)
            inset = float(gcmd.get_float("INSET", self.inset))
            baseline_dwell = float(gcmd.get_float("BASELINE_DWELL", 5.0))
            lpf_cutoff_hz = float(gcmd.get_float("LPF_CUTOFF_HZ", 150.0))
            noise_penalty = gcmd.get("NOISE_PENALTY", "none").lower()
            raw = bool(gcmd.get_int("RAW", 0))

            # ACCEL_CHIP is hard to pass when it contains spaces (e.g. "lis2dw e0_lis2dw").
            # Default resolves from [resonance_tester].
            accel_chip = gcmd.get("ACCEL_CHIP", "default")
            if accel_chip == "default":
                # Backwards compat with older docs.
                accel_chip = gcmd.get("ACCELEROMETER", "default")

            log_path = gcmd.get("LOG_PATH", U1_LOG_ROOT)
            if log_path.rstrip("/") != U1_LOG_ROOT:
                raise self.printer.command_error(
                    f"LOG_PATH must be '{U1_LOG_ROOT}' on Snapmaker U1"
                )

            # Keep compatibility with old knob, but do not support it in U1 mode.
            apply_static = bool(gcmd.get_int("APPLY_STATIC", 0))
            if apply_static:
                raise self.printer.command_error("APPLY_STATIC=1 is not implemented yet")

            self.iterations = gcmd.get_int("ITERATIONS", 1)

            self.chopper_tune(
                axis=axis,
                quick=quick,
                home=home,
                tool=tool,
                accel_chip=accel_chip,
                inset=inset,
                baseline_dwell=baseline_dwell,
                lpf_cutoff_hz=lpf_cutoff_hz,
                noise_penalty=noise_penalty,
                raw=raw,
                log_path=log_path,
            )
        except Exception:
            self.gcode.respond_info(traceback.format_exc())

    def cmd_chopper_tune_debug(self, gcmd: GCodeCommand) -> bool:
        """Development debug tool.

        Args:
            gcmd (GCodeCommand): The G-Code command.

        Returns:
            bool: True if command completed successfully, False otherwise.
        """
        try:
            self.gcode.respond_info(f"x stepper count: {self.get_stepper_count('x')}")
            self.gcode.respond_info(f"y stepper count: {self.get_stepper_count('y')}")
            self.gcode.respond_info(f"z stepper count: {self.get_stepper_count('z')}")
        except Exception as e:
            self.gcode.respond_info(traceback.format_exc(e))


def load_config(config: ConfigWrapper) -> ChopperTune:
    """Load the ChopperTune config prefix.

    Args:
        config (ConfigWrapper): The config wrapper.

    Returns:
        ChopperTune: The ChopperTune instance.
    """
    return ChopperTune(config)


def load_config_prefix(config: ConfigWrapper) -> ChopperTune:
    """Load the ChopperTune config prefix.

    Args:
        config (ConfigWrapper): The config wrapper.

    Returns:
        ChopperTune: The ChopperTune instance.
    """
    return ChopperTune(config)
