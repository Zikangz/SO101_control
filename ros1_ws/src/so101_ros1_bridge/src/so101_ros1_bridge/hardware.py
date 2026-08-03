import math
import glob
import sys
import threading
import time
from pathlib import Path


MODEL_RESOLUTION = 4096
ENCODE_SIGN_BIT = {
    "Goal_Position": 15,
    "Present_Position": 15,
    "Present_Velocity": 15,
    "Present_Load": 15,
    "Present_Current": 15,
    "Goal_Velocity": 15,
    "Homing_Offset": 11,
}


def encode_sign_magnitude(value, sign_bit_index):
    value = int(value)
    max_magnitude = (1 << sign_bit_index) - 1
    magnitude = abs(value)
    if magnitude > max_magnitude:
        raise ValueError("Magnitude %d exceeds %d" % (magnitude, max_magnitude))
    direction_bit = 1 if value < 0 else 0
    return (direction_bit << sign_bit_index) | magnitude


def decode_sign_magnitude(encoded_value, sign_bit_index):
    encoded_value = int(encoded_value)
    direction_bit = (encoded_value >> sign_bit_index) & 1
    magnitude_mask = (1 << sign_bit_index) - 1
    magnitude = encoded_value & magnitude_mask
    return -magnitude if direction_bit else magnitude


class BaseSO101Backend:
    def connect(self):
        raise NotImplementedError

    def read_positions(self):
        raise NotImplementedError

    def write_positions(self, positions):
        raise NotImplementedError

    def read_diagnostics(self):
        return {}

    def relax(self):
        pass

    def close(self):
        pass


class MockSO101Backend(BaseSO101Backend):
    def __init__(self, joint_order, home_positions, max_velocity):
        self.joint_order = list(joint_order)
        self.positions = {name: float(home_positions.get(name, 0.0)) for name in self.joint_order}
        self.goals = dict(self.positions)
        self.max_velocity = dict(max_velocity)
        self.last_time = time.time()
        self.connected = False

    def connect(self):
        self.connected = True

    def read_positions(self):
        now = time.time()
        dt = max(0.0, now - self.last_time)
        self.last_time = now
        for name in self.joint_order:
            goal = self.goals.get(name, self.positions[name])
            current = self.positions[name]
            max_step = abs(float(self.max_velocity.get(name, 1.0))) * dt
            err = goal - current
            if abs(err) <= max_step:
                self.positions[name] = goal
            else:
                self.positions[name] = current + math.copysign(max_step, err)
        return dict(self.positions)

    def write_positions(self, positions):
        for name, value in positions.items():
            if name in self.positions:
                self.goals[name] = float(value)

    def read_diagnostics(self):
        return {
            name: {
                "position": float(self.positions[name]),
                "velocity_raw": 0,
                "load_raw": 0,
                "voltage_v": 12.0,
                "temperature_c": 25,
                "current_raw": 0,
                "current_ma": 0.0,
                "moving": abs(self.goals.get(name, self.positions[name]) - self.positions[name]) > 1e-6,
            }
            for name in self.joint_order
        }


class MujocoSO101Backend(BaseSO101Backend):
    """MuJoCo-backed SO101 joint-position simulator for the ROS1 bridge."""

    def __init__(self, joint_order, home_positions, model_path="", substeps=10):
        self.joint_order = list(joint_order)
        self.home_positions = {name: float(home_positions.get(name, 0.0)) for name in self.joint_order}
        self.model_path = str(model_path or self._default_model_path())
        self.substeps = max(1, int(substeps))
        self.goals = dict(self.home_positions)
        self.mujoco = None
        self.model = None
        self.data = None
        self.qpos_addr = {}
        self.qvel_addr = {}
        self.joint_ranges = {}
        self.actuator_ids = {}
        self.connected = False

    def _default_model_path(self):
        path = Path(__file__).resolve()
        for parent in path.parents:
            candidate = parent / "assets" / "so101" / "scene.xml"
            if candidate.exists():
                return candidate
        return Path("/home/bot/research/so101_mujoco_tracking/assets/so101/scene.xml")

    def _import_mujoco(self):
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError(
                "Missing mujoco for ROS Python. Install it in the ROS1 Python environment, "
                "or run this backend from an environment where 'python3 -c \"import mujoco\"' works."
            ) from exc
        return mujoco

    def connect(self):
        self.mujoco = self._import_mujoco()
        model_path = Path(self.model_path).expanduser()
        if not model_path.exists():
            raise RuntimeError("MuJoCo model path does not exist: %s" % model_path)
        self.model = self.mujoco.MjModel.from_xml_path(str(model_path))
        self.data = self.mujoco.MjData(self.model)

        missing = []
        for name in self.joint_order:
            joint_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, name)
            actuator_id = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if joint_id < 0:
                missing.append("joint:%s" % name)
                continue
            if actuator_id < 0:
                missing.append("actuator:%s" % name)
                continue
            self.qpos_addr[name] = int(self.model.jnt_qposadr[joint_id])
            self.qvel_addr[name] = int(self.model.jnt_dofadr[joint_id])
            self.joint_ranges[name] = tuple(float(v) for v in self.model.jnt_range[joint_id])
            self.actuator_ids[name] = int(actuator_id)
        if missing:
            raise RuntimeError("MuJoCo model is missing SO101 names: %s" % ", ".join(missing))

        for name, value in self.home_positions.items():
            if name not in self.qpos_addr:
                continue
            mj_value = self._ros_to_mujoco(name, value)
            self.data.qpos[self.qpos_addr[name]] = mj_value
            self.data.ctrl[self.actuator_ids[name]] = mj_value
            self.goals[name] = value
        self.mujoco.mj_forward(self.model, self.data)
        self.connected = True

    def _range_for_joint(self, joint):
        if joint in self.actuator_ids:
            actuator_id = self.actuator_ids[joint]
            ctrlrange = self.model.actuator_ctrlrange[actuator_id]
            if float(ctrlrange[1]) > float(ctrlrange[0]):
                return float(ctrlrange[0]), float(ctrlrange[1])
        return self.joint_ranges.get(joint, (0.0, 1.0))

    def _ros_to_mujoco(self, joint, position):
        value = float(position)
        if joint == "gripper":
            lo, hi = self._range_for_joint(joint)
            value = max(0.0, min(1.0, value))
            return lo + value * (hi - lo)
        return value

    def _mujoco_to_ros(self, joint, position):
        value = float(position)
        if joint == "gripper":
            lo, hi = self._range_for_joint(joint)
            if hi <= lo:
                return 0.0
            return max(0.0, min(1.0, (value - lo) / (hi - lo)))
        return value

    def _step(self):
        if not self.connected:
            return
        for _ in range(self.substeps):
            self.mujoco.mj_step(self.model, self.data)

    def read_positions(self):
        self._step()
        return {
            name: self._mujoco_to_ros(name, self.data.qpos[self.qpos_addr[name]])
            for name in self.joint_order
            if name in self.qpos_addr
        }

    def write_positions(self, positions):
        for name, value in positions.items():
            if name not in self.actuator_ids:
                continue
            self.goals[name] = float(value)
            self.data.ctrl[self.actuator_ids[name]] = self._ros_to_mujoco(name, value)

    def read_diagnostics(self):
        result = {}
        for name in self.joint_order:
            if name not in self.qpos_addr:
                continue
            position = self._mujoco_to_ros(name, self.data.qpos[self.qpos_addr[name]])
            velocity = float(self.data.qvel[self.qvel_addr[name]])
            result[name] = {
                "position": position,
                "raw_position": "",
                "velocity_raw": velocity,
                "load_raw": 0,
                "voltage_v": "",
                "temperature_c": "",
                "current_raw": "",
                "current_ma": "",
                "moving": abs(position - self.goals.get(name, position)) > 1e-4,
            }
        return result


class FeetechSO101Backend(BaseSO101Backend):
    CONTROL_TABLE = {
        "ID": (5, 1),
        "Baud_Rate": (6, 1),
        "Return_Delay_Time": (7, 1),
        "Min_Position_Limit": (9, 2),
        "Max_Position_Limit": (11, 2),
        "P_Coefficient": (21, 1),
        "D_Coefficient": (22, 1),
        "I_Coefficient": (23, 1),
        "Homing_Offset": (31, 2),
        "Operating_Mode": (33, 1),
        "Max_Torque_Limit": (16, 2),
        "Protection_Current": (28, 2),
        "Overload_Torque": (36, 1),
        "Torque_Enable": (40, 1),
        "Acceleration": (41, 1),
        "Goal_Position": (42, 2),
        "Present_Position": (56, 2),
        "Present_Velocity": (58, 2),
        "Present_Load": (60, 2),
        "Present_Voltage": (62, 1),
        "Present_Temperature": (63, 1),
        "Moving": (66, 1),
        "Present_Current": (69, 2),
        "Maximum_Acceleration": (85, 1),
    }

    def __init__(
        self,
        port,
        baudrate,
        joint_order,
        motor_ids,
        calibration,
        disable_torque_on_relax=True,
        servo_pid=None,
        servo_speed=800,
        servo_acceleration=50,
        servo_maximum_acceleration=None,
        skip_duplicate_writes=True,
    ):
        self.port = port
        self.baudrate = int(baudrate)
        self.joint_order = list(joint_order)
        self.motor_ids = {name: int(motor_ids[name]) for name in self.joint_order}
        self.calibration = calibration
        self.disable_torque_on_relax = bool(disable_torque_on_relax)
        self.servo_pid = dict(servo_pid or {})
        self.servo_speed = int(servo_speed)
        self.servo_acceleration = int(servo_acceleration)
        self.servo_maximum_acceleration = (
            None if servo_maximum_acceleration is None else int(servo_maximum_acceleration)
        )
        self.skip_duplicate_writes = bool(skip_duplicate_writes)
        self.last_goal_raw = {}
        self.scs = None
        self.port_handler = None
        self.packet_handler = None
        self.sdk_api = None
        self.io_lock = threading.RLock()

    def _project_root(self):
        path = Path(__file__).resolve()
        for parent in path.parents:
            if (parent / "third_party").is_dir() and (parent / "ros1_ws").is_dir():
                return parent
        return Path("/home/zzk/ZZK/SO101")

    def _import_scservo_sdk(self):
        root = self._project_root()
        preferred = [
            root / "third_party" / "Seeed_RoboController",
            root / "Seeed_RoboController",
        ]
        for candidate in reversed(preferred):
            if (candidate / "scservo_sdk").is_dir():
                sys.path.insert(0, str(candidate))

        existing = sys.modules.get("scservo_sdk")
        if existing is not None:
            loaded_from = str(getattr(existing, "__file__", ""))
            if not any(str(candidate) in loaded_from for candidate in preferred):
                for name in list(sys.modules):
                    if name == "scservo_sdk" or name.startswith("scservo_sdk."):
                        del sys.modules[name]

        import scservo_sdk as scs
        return scs

    def _available_ports(self):
        ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        by_id = sorted(glob.glob("/dev/serial/by-id/*"))
        if by_id:
            ports.extend("%s -> %s" % (path, Path(path).resolve()) for path in by_id)
        return ports

    def connect(self):
        try:
            scs = self._import_scservo_sdk()
        except ImportError as exc:
            raise RuntimeError(
                "Missing scservo_sdk. Install with: python3 -m pip install 'feetech-servo-sdk>=1.0.0,<2.0.0'"
            ) from exc

        self.scs = scs
        self.port_handler = scs.PortHandler(self.port)
        if hasattr(scs, "sms_sts"):
            self.packet_handler = scs.sms_sts(self.port_handler)
            self.sdk_api = "bound"
        else:
            self.packet_handler = scs.PacketHandler(0)
            self.sdk_api = "packet"

        if not Path(self.port).exists():
            available = ", ".join(self._available_ports()) or "none"
            raise RuntimeError("Feetech port does not exist: %s. Available serial ports: %s" % (self.port, available))
        try:
            opened = self.port_handler.openPort()
        except Exception as exc:
            raise RuntimeError("Failed to open Feetech port %s: %s" % (self.port, exc)) from exc
        if not opened:
            raise RuntimeError("Failed to open Feetech port: %s" % self.port)
        try:
            baud_ok = self.port_handler.setBaudRate(self.baudrate)
        except Exception as exc:
            raise RuntimeError("Failed to set Feetech baudrate %s on %s: %s" % (self.baudrate, self.port, exc)) from exc
        if not baud_ok:
            raise RuntimeError("Failed to set Feetech baudrate: %s" % self.baudrate)

        self._configure_motors()

    def _pid_value(self, joint, key, default):
        joint_pid = self.servo_pid.get(joint, {})
        default_pid = self.servo_pid.get("default", {})
        if isinstance(joint_pid, dict) and key in joint_pid:
            return int(joint_pid[key])
        if isinstance(default_pid, dict) and key in default_pid:
            return int(default_pid[key])
        if key in self.servo_pid:
            return int(self.servo_pid[key])
        return int(default)

    def _configure_motors(self):
        for joint in self.joint_order:
            self._write(joint, "Torque_Enable", 0)
            self._write(joint, "Return_Delay_Time", 0)
            self._write(joint, "Operating_Mode", 0)
            self._write(joint, "P_Coefficient", self._pid_value(joint, "p", 16))
            self._write(joint, "I_Coefficient", self._pid_value(joint, "i", 0))
            self._write(joint, "D_Coefficient", self._pid_value(joint, "d", 32))
            if self.servo_maximum_acceleration is not None:
                self._write(joint, "Maximum_Acceleration", self.servo_maximum_acceleration)
            self._write(joint, "Acceleration", self.servo_acceleration)
            if joint == "gripper":
                self._write(joint, "Max_Torque_Limit", 500)
                self._write(joint, "Protection_Current", 250)
                self._write(joint, "Overload_Torque", 25)
            self._write(joint, "Torque_Enable", 1)

    def _read(self, joint, data_name):
        address, length = self.CONTROL_TABLE[data_name]
        motor_id = self.motor_ids[joint]
        for attempt in range(3):
            with self.io_lock:
                if length == 1:
                    if self.sdk_api == "bound":
                        value, comm, err = self.packet_handler.read1ByteTxRx(motor_id, address)
                    else:
                        value, comm, err = self.packet_handler.read1ByteTxRx(self.port_handler, motor_id, address)
                elif length == 2:
                    if self.sdk_api == "bound":
                        value, comm, err = self.packet_handler.read2ByteTxRx(motor_id, address)
                    else:
                        value, comm, err = self.packet_handler.read2ByteTxRx(self.port_handler, motor_id, address)
                else:
                    if self.sdk_api == "bound":
                        value, comm, err = self.packet_handler.read4ByteTxRx(motor_id, address)
                    else:
                        value, comm, err = self.packet_handler.read4ByteTxRx(self.port_handler, motor_id, address)
            if comm == self.scs.COMM_SUCCESS:
                break
            if comm == getattr(self.scs, "COMM_PORT_BUSY", None) and attempt < 2:
                if self.port_handler is not None:
                    self.port_handler.is_using = False
                time.sleep(0.03)
                continue
            break
        if comm != self.scs.COMM_SUCCESS:
            raise RuntimeError("Read %s from %s failed: %s" % (data_name, joint, self.packet_handler.getTxRxResult(comm)))
        if err:
            raise RuntimeError("Read %s from %s returned servo error: %s" % (data_name, joint, err))
        if data_name in ENCODE_SIGN_BIT:
            value = decode_sign_magnitude(value, ENCODE_SIGN_BIT[data_name])
        return value

    def _write(self, joint, data_name, value):
        address, length = self.CONTROL_TABLE[data_name]
        motor_id = self.motor_ids[joint]
        raw = int(value)
        if data_name in ENCODE_SIGN_BIT:
            raw = encode_sign_magnitude(raw, ENCODE_SIGN_BIT[data_name])
        data = self._split_bytes(raw, length)
        for attempt in range(3):
            with self.io_lock:
                if self.sdk_api == "bound":
                    comm, err = self.packet_handler.writeTxRx(motor_id, address, length, data)
                else:
                    comm, err = self.packet_handler.writeTxRx(self.port_handler, motor_id, address, length, data)
            if comm == self.scs.COMM_SUCCESS:
                break
            if comm == getattr(self.scs, "COMM_PORT_BUSY", None) and attempt < 2:
                if self.port_handler is not None:
                    self.port_handler.is_using = False
                time.sleep(0.03)
                continue
            break
        if comm != self.scs.COMM_SUCCESS:
            raise RuntimeError("Write %s to %s failed: %s" % (data_name, joint, self.packet_handler.getTxRxResult(comm)))
        if err:
            raise RuntimeError("Write %s to %s returned servo error: %s" % (data_name, joint, err))
        time.sleep(0.003)

    def _lo_byte(self, value):
        if hasattr(self.scs, "SCS_LOBYTE"):
            return self.scs.SCS_LOBYTE(value)
        return self.packet_handler.scs_lobyte(value)

    def _hi_byte(self, value):
        if hasattr(self.scs, "SCS_HIBYTE"):
            return self.scs.SCS_HIBYTE(value)
        return self.packet_handler.scs_hibyte(value)

    def _lo_word(self, value):
        if hasattr(self.scs, "SCS_LOWORD"):
            return self.scs.SCS_LOWORD(value)
        return self.packet_handler.scs_loword(value)

    def _hi_word(self, value):
        if hasattr(self.scs, "SCS_HIWORD"):
            return self.scs.SCS_HIWORD(value)
        return self.packet_handler.scs_hiword(value)

    def _split_bytes(self, value, length):
        if length == 1:
            return [value & 0xFF]
        if length == 2:
            return [self._lo_byte(value), self._hi_byte(value)]
        if length == 4:
            return [
                self._lo_byte(self._lo_word(value)),
                self._hi_byte(self._lo_word(value)),
                self._lo_byte(self._hi_word(value)),
                self._hi_byte(self._hi_word(value)),
            ]
        raise ValueError("Unsupported byte length: %s" % length)

    def _joint_to_raw(self, joint, position):
        cal = self.calibration.get(joint, {})
        range_min = int(cal.get("range_min", 0))
        range_max = int(cal.get("range_max", MODEL_RESOLUTION - 1))
        homing_offset = int(cal.get("homing_offset", (range_min + range_max) / 2))
        if joint == "gripper":
            if range_max <= range_min:
                range_min = max(0, homing_offset - 500)
                range_max = min(MODEL_RESOLUTION - 1, homing_offset + 500)
            bounded = max(0.0, min(1.0, float(position)))
            return int(range_min + bounded * (range_max - range_min))
        return int(float(position) * (MODEL_RESOLUTION - 1) / (2.0 * math.pi) + homing_offset)

    def _raw_to_joint(self, joint, raw):
        cal = self.calibration.get(joint, {})
        range_min = int(cal.get("range_min", 0))
        range_max = int(cal.get("range_max", MODEL_RESOLUTION - 1))
        homing_offset = int(cal.get("homing_offset", (range_min + range_max) / 2))
        if joint == "gripper":
            if range_max <= range_min:
                range_min = max(0, homing_offset - 500)
                range_max = min(MODEL_RESOLUTION - 1, homing_offset + 500)
            return max(0.0, min(1.0, (float(raw) - range_min) / (range_max - range_min)))
        return (float(raw) - homing_offset) * (2.0 * math.pi) / (MODEL_RESOLUTION - 1)

    def read_positions(self):
        result = {}
        for joint in self.joint_order:
            raw = self._read(joint, "Present_Position")
            result[joint] = self._raw_to_joint(joint, raw)
        return result

    def read_diagnostics(self):
        result = {}
        for joint in self.joint_order:
            raw_position = self._read(joint, "Present_Position")
            velocity = self._read(joint, "Present_Velocity")
            load = self._read(joint, "Present_Load")
            voltage = self._read(joint, "Present_Voltage")
            temperature = self._read(joint, "Present_Temperature")
            moving = self._read(joint, "Moving")
            current = self._read(joint, "Present_Current")
            result[joint] = {
                "position": self._raw_to_joint(joint, raw_position),
                "raw_position": raw_position,
                "velocity_raw": velocity,
                "load_raw": load,
                "voltage_v": float(voltage) / 10.0,
                "temperature_c": int(temperature),
                "current_raw": current,
                "current_ma": float(current) * 6.5,
                "moving": bool(moving),
            }
            time.sleep(0.003)
        return result

    def write_positions(self, positions):
        commanded = []
        for joint, position in positions.items():
            if joint not in self.motor_ids:
                continue
            raw = self._joint_to_raw(joint, position)
            if self.skip_duplicate_writes and self.last_goal_raw.get(joint) == raw:
                continue
            commanded.append((joint, raw))
        if not commanded:
            return

        if self.sdk_api == "bound" and hasattr(self.packet_handler, "SyncWritePosEx"):
            sync_error = ""
            with self.io_lock:
                try:
                    self.packet_handler.groupSyncWrite.clearParam()
                    for joint, raw in commanded:
                        added = self.packet_handler.SyncWritePosEx(
                            self.motor_ids[joint],
                            raw,
                            self.servo_speed,
                            self.servo_acceleration,
                        )
                        if not added:
                            raise RuntimeError("SyncWrite addParam failed for %s" % joint)
                    comm = self.packet_handler.groupSyncWrite.txPacket()
                    self.packet_handler.groupSyncWrite.clearParam()
                    if comm == self.scs.COMM_SUCCESS:
                        for joint, raw in commanded:
                            self.last_goal_raw[joint] = raw
                        return
                    raise RuntimeError("SyncWritePosEx failed: %s" % self.packet_handler.getTxRxResult(comm))
                except Exception as exc:
                    sync_error = str(exc)
                    self.packet_handler.groupSyncWrite.clearParam()
            if sync_error:
                # Fall back to individual WritePosEx below. Some SDK variants
                # expose SyncWritePosEx but do not support it reliably.
                pass

        if self.sdk_api == "bound" and hasattr(self.packet_handler, "WritePosEx"):
            for joint, raw in commanded:
                motor_id = self.motor_ids[joint]
                for attempt in range(3):
                    with self.io_lock:
                        comm, err = self.packet_handler.WritePosEx(
                            motor_id,
                            raw,
                            self.servo_speed,
                            self.servo_acceleration,
                        )
                    if comm == self.scs.COMM_SUCCESS:
                        break
                    if comm == getattr(self.scs, "COMM_PORT_BUSY", None) and attempt < 2:
                        if self.port_handler is not None:
                            self.port_handler.is_using = False
                        time.sleep(0.03)
                        continue
                    break
                if comm != self.scs.COMM_SUCCESS:
                    raise RuntimeError("WritePosEx to %s failed: %s" % (joint, self.packet_handler.getTxRxResult(comm)))
                if err:
                    raise RuntimeError("WritePosEx to %s returned servo error: %s" % (joint, err))
                self.last_goal_raw[joint] = raw
            return

        for joint, raw in commanded:
            self._write(joint, "Goal_Position", raw)
            self.last_goal_raw[joint] = raw

    def relax(self):
        if not self.disable_torque_on_relax:
            return
        for joint in self.joint_order:
            try:
                self._write(joint, "Torque_Enable", 0)
            except Exception:
                pass

    def close(self):
        if self.port_handler is None:
            return
        if self.disable_torque_on_relax:
            self.relax()
        try:
            self.port_handler.closePort()
        except Exception:
            pass
        finally:
            self.port_handler = None
