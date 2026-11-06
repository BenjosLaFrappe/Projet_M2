import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import sys
import time
import threading
import ast
import json

import paho.mqtt.client as paho

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2
from kortex_api.Exceptions.KServerException import KServerException


# ============================================================
# PARAMS
# ============================================================
TIMEOUT_DURATION = 40

GIVE_X_NEG_OFFSET_M = 0.0
TOOL_X_ADD_M = 0.05  # 5 cm

DIST_ENTER_M = 0.04
DIST_EXIT_M  = 0.10
HOLD_OK_TIME_S = 0.6

CTRL_HZ = 30.0
CTRL_DT = 1.0 / CTRL_HZ
KP_LIN = 1.2
V_MAX = 0.10
FILTER_TAU_S = 0.25

MIN_X, MAX_X = -0.10, 0.90
MIN_Y, MAX_Y = -0.90, 0.90
MIN_Z, MAX_Z =  0.02, 1.10

APPROACH_TIMEOUT_S = 25.0


# ============================================================
# EMERGENCY STOP (IHM)
# ============================================================
MQTT_SCADA_CMD_TOPIC = "scada/cmd"
EMERGENCY_PAYLOAD = "emergency"
_stop_event = threading.Event()


class StopRequested(Exception):
    pass


def _payload_is_emergency(payload_str):
    s = (payload_str or "").strip().lower()
    if s == EMERGENCY_PAYLOAD:
        return True
    if "emergency" in s:
        return True
    if s in ("estop", "e-stop", "e stop"):
        return True
    return False


def emergency_requested():
    return _stop_event.is_set()


def check_emergency_and_abort(base=None, where=""):
    """
    À appeler régulièrement dans les boucles.
    Si emergency -> stop bras + exception StopRequested.
    """
    if emergency_requested():
        try:
            if base is not None:
                try:
                    stop_twist_hard(base)
                except Exception:
                    try:
                        stop_robot(base)
                    except Exception:
                        pass
        finally:
            raise StopRequested("Emergency stop demandé{}".format(" ({})".format(where) if where else ""))


# ============================================================
# MQTT (object + hand + emergency)
# ============================================================
MQTT_BROKER_IP = "127.0.0.1"
MQTT_PORT = 1883
MQTT_OBJECT_TOPIC = "camera/object"
MQTT_HAND_TOPIC = "camera/hand"

MQTT_STATUS_TOPIC = "kinova_status"
_last_object_xy = None   # tuple (x, y) pixels (cam 2D)
_last_hand_xyz = None    # tuple (x, y, z) mètres (cam 3D)

_mqtt_client = None
_mqtt_lock = threading.Lock()


def _on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("[MQTT] Connecté. Abonnements:", MQTT_OBJECT_TOPIC, "et", MQTT_HAND_TOPIC, "et", MQTT_SCADA_CMD_TOPIC)
        client.subscribe(MQTT_OBJECT_TOPIC)
        client.subscribe(MQTT_HAND_TOPIC)
        # Emergency stop IHM
        client.subscribe(MQTT_SCADA_CMD_TOPIC)
    else:
        print("[MQTT] Échec connexion (rc={})".format(rc))


def _on_message(client, userdata, msg):
    """
    Supporte 3 topics :
      - camera/object : payload dict {'center_x':..., 'center_y':...} (souvent envoyé via ast)
      - camera/hand   : payload "no_hand" OU JSON {"x":..,"y":..,"z":..}
      - scada/cmd     : payload "emergency" => stop global
    """
    global _last_object_xy, _last_hand_xyz

    topic = getattr(msg, "topic", "")
    payload_str = msg.payload.decode(errors="replace").strip()
    payload_lower = payload_str.lower()

    # -------------------------
    # Emergency stop (IHM)
    # -------------------------
    if topic == MQTT_SCADA_CMD_TOPIC:
        if _payload_is_emergency(payload_str):
            print("[MQTT] *** EMERGENCY reçu sur {} payload='{}' ***".format(MQTT_SCADA_CMD_TOPIC, payload_str))
            _stop_event.set()
        return

    if payload_lower == "ready":
        return

    # -------------------------
    # Topic OBJECT (2D)
    # -------------------------
    if topic == MQTT_OBJECT_TOPIC:
        if payload_lower == "no_object":
            with _mqtt_lock:
                _last_object_xy = None
            return

        try:
            data = ast.literal_eval(payload_str)
        except (ValueError, SyntaxError):
            return

        if isinstance(data, dict) and ("center_x" in data) and ("center_y" in data):
            try:
                x = float(data["center_x"])
                y = float(data["center_y"])
            except (TypeError, ValueError):
                return
            with _mqtt_lock:
                _last_object_xy = (x, y)
        return

    # -------------------------
    # Topic HAND (3D)
    # -------------------------
    if topic == MQTT_HAND_TOPIC:
        if payload_lower == "no_hand":
            with _mqtt_lock:
                _last_hand_xyz = None
            return

        data = None
        try:
            data = json.loads(payload_str)
        except Exception:
            try:
                data = ast.literal_eval(payload_str)
            except Exception:
                return

        if not isinstance(data, dict):
            return
        if ("x" not in data) or ("y" not in data) or ("z" not in data):
            return

        try:
            x = float(data["x"])
            y = float(data["y"])
            z = float(data["z"])
        except (TypeError, ValueError):
            return

        with _mqtt_lock:
            _last_hand_xyz = (x, y, z)
        return


def start_mqtt_receiver():
    global _mqtt_client

    if _mqtt_client is not None:
        try:
            if getattr(_mqtt_client, "is_connected", lambda: False)():
                return
        except Exception:
            pass

    try:
        _mqtt_client = paho.Client(paho.CallbackAPIVersion.VERSION2)
    except Exception:
        _mqtt_client = paho.Client()

    _mqtt_client.on_connect = _on_connect
    _mqtt_client.on_message = _on_message

    print("[MQTT] Connexion à {}:{} ...".format(MQTT_BROKER_IP, MQTT_PORT))
    _mqtt_client.connect(MQTT_BROKER_IP, MQTT_PORT, 60)
    _mqtt_client.loop_start()


def stop_mqtt_receiver():
    global _mqtt_client
    if _mqtt_client is None:
        return
    try:
        try:
            if getattr(_mqtt_client, "is_connected", lambda: True)():
                _mqtt_client.loop_stop()
                _mqtt_client.disconnect()
            else:
                _mqtt_client.loop_stop()
        except Exception:
            try:
                _mqtt_client.loop_stop()
            except Exception:
                pass
            try:
                _mqtt_client.disconnect()
            except Exception:
                pass
    finally:
        _mqtt_client = None


def publish_kinova_status(status):
    global _mqtt_client
    if _mqtt_client is None:
        return
    try:
        _mqtt_client.publish(MQTT_STATUS_TOPIC, str(status))
    except Exception:
        pass


def get_object_coord():
    with _mqtt_lock:
        if _last_object_xy is None:
            return [0.0, 0.0]
        return [float(_last_object_xy[0]), float(_last_object_xy[1])]


def get_hand_coord():
    with _mqtt_lock:
        if _last_hand_xyz is None:
            return [0.0, 0.0, 0.0]
        return [float(_last_hand_xyz[0]), float(_last_hand_xyz[1]), float(_last_hand_xyz[2])]


# ============================================================
# TRANSFORMATION CAMERA 2D -> BASE KINOVA
# ============================================================
_AFF_PIX_TO_KIN_CM = (
    (0.125271031,  0.00488696529, -21.9635179),
    (-0.00656357677, 0.124287755, -91.0020926),
)

def camera_pixels_to_kinova_xy_m(cx, cy):
    px = float(cx)
    py = float(cy)

    x_cm = (_AFF_PIX_TO_KIN_CM[0][0] * px) + (_AFF_PIX_TO_KIN_CM[0][1] * py) + (_AFF_PIX_TO_KIN_CM[0][2])
    y_cm = (_AFF_PIX_TO_KIN_CM[1][0] * px) + (_AFF_PIX_TO_KIN_CM[1][1] * py) + (_AFF_PIX_TO_KIN_CM[1][2])

    return (x_cm / 100.0, y_cm / 100.0)


# ============================================================
# TRANSFORMATION CAMERA 3D -> BASE KINOVA
# ============================================================
_CAM_A = (
    (0.92638889, -0.02326389, -0.02916667),
    (0.00986111,  0.60420139,  0.72541667),
    (0.11076389,  0.79314236, -0.59479167),
)
_CAM_T = (0.53871528, -0.38855903, 0.54544965)

def camera3d_coordinates_to_kinova_xyz_m(x, y, z):
    xc = float(x)
    yc = float(y)
    zc = float(z)
    X = _CAM_A[0][0]*xc + _CAM_A[0][1]*yc + _CAM_A[0][2]*zc + _CAM_T[0]
    Y = _CAM_A[1][0]*xc + _CAM_A[1][1]*yc + _CAM_A[1][2]*zc + _CAM_T[1]
    Z = _CAM_A[2][0]*xc + _CAM_A[2][1]*yc + _CAM_A[2][2]*zc + _CAM_T[2]
    return X, Y, Z

def _self_test_transform():
    tests = [
        ((0.05, -0.02, 0.36), (0.575, -0.139, 0.321)),
        ((0.15,  0.02, 0.59), (0.660,  0.053, 0.227)),
        ((0.17, -0.14, 0.53), (0.684, -0.087, 0.138)),
        ((0.20,  0.10, 0.40), (0.710, -0.036, 0.409)),
    ]
    print("[CALIB] Self-test transform (cam->kinova):")
    for cam, kin_ref in tests:
        kin = camera3d_coordinates_to_kinova_xyz_m(cam[0], cam[1], cam[2])
        dx = kin[0] - kin_ref[0]
        dy = kin[1] - kin_ref[1]
        dz = kin[2] - kin_ref[2]
        print("  cam {} -> kin {:.3f},{:.3f},{:.3f} | err {:.6f},{:.6f},{:.6f}".format(
            cam, kin[0], kin[1], kin[2], dx, dy, dz
        ))


# ============================================================
# KINOVA HELPERS
# ============================================================
def check_for_end_or_abort(e):
    def check(notification, e=e):
        print("EVENT : " + Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check


def ensure_single_level_servoing(base):
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)


def stop_robot(base):
    try:
        base.Stop()
    except Exception:
        pass


def send_twist_base(base, vx, vy, vz):
    command = Base_pb2.TwistCommand()
    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
    command.duration = 0

    twist = command.twist
    twist.linear_x = float(vx)
    twist.linear_y = float(vy)
    twist.linear_z = float(vz)

    twist.angular_x = 0.0
    twist.angular_y = 0.0
    twist.angular_z = 0.0

    base.SendTwistCommand(command)


def stop_twist_hard(base):
    try:
        for _ in range(3):
            send_twist_base(base, 0.0, 0.0, 0.0)
            time.sleep(0.02)
        base.Stop()
        for _ in range(3):
            send_twist_base(base, 0.0, 0.0, 0.0)
            time.sleep(0.02)
    except Exception:
        pass


def _wait_event_or_emergency(e, timeout_s, base, where=""):
    """
    Remplace e.wait(timeout) par une attente interruptible (emergency).
    """
    t_end = time.time() + float(timeout_s)
    while True:
        check_emergency_and_abort(base, where=where)
        if e.is_set():
            return True
        remaining = t_end - time.time()
        if remaining <= 0.0:
            return False
        e.wait(min(0.1, remaining))


def move_to_known_position(base, known_pose):
    ensure_single_level_servoing(base)

    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)

    action_handle = None
    for action in action_list.action_list:
        if action.name == known_pose:
            print("moving to : ", known_pose)
            action_handle = action.handle

    if action_handle is None:
        print("Action '{}' introuvable".format(known_pose))
        return False

    e = threading.Event()
    notif_handle = base.OnNotificationActionTopic(check_for_end_or_abort(e), Base_pb2.NotificationOptions())

    pose_key = str(known_pose).strip().lower()
    if pose_key in ("attaque", "transition", "give", "zero"):
        publish_kinova_status(pose_key)

    base.ExecuteActionFromReference(action_handle)

    finished = _wait_event_or_emergency(e, TIMEOUT_DURATION, base, where="move_to_known_position({})".format(known_pose))
    base.Unsubscribe(notif_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished


class GripperCommand:
    def __init__(self, base):
        self.base = base

    def Send_Gripper_Command_Close(self):
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        print("Closing gripper using speed command...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = -1
        self.base.SendGripperCommand(gripper_command)
        time.sleep(0.5)

        gripper_request = Base_pb2.GripperRequest()
        gripper_request.mode = Base_pb2.GRIPPER_SPEED
        while True:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len(gripper_measure.finger):
                if abs(gripper_measure.finger[0].value) < 0.04:
                    print("gripper closed")
                    break
            else:
                break

    def Send_Gripper_Command_Open(self):
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        print("Opening gripper using speed command...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = 1
        self.base.SendGripperCommand(gripper_command)

        gripper_request = Base_pb2.GripperRequest()
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        while True:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len(gripper_measure.finger):
                if abs(gripper_measure.finger[0].value) < 0.01:
                    print("gripper opened")
                    break
            else:
                break


def inverse_kinematics(base, desired_pose):
    try:
        measured = base.GetMeasuredJointAngles()
    except KServerException as ex:
        print("Unable to get joint angles for IK guess:", ex)
        return False

    ik = Base_pb2.IKData()
    ik.cartesian_pose.x = float(desired_pose[0])
    ik.cartesian_pose.y = float(desired_pose[1])
    ik.cartesian_pose.z = float(desired_pose[2])
    ik.cartesian_pose.theta_x = float(desired_pose[3])
    ik.cartesian_pose.theta_y = float(desired_pose[4])
    ik.cartesian_pose.theta_z = float(desired_pose[5])

    for joint_angle in measured.joint_angles:
        j = ik.guess.joint_angles.add()
        j.value = joint_angle.value

    try:
        print("Computing Inverse Kinematics using joint angles guess and target pose...")
        computed = base.ComputeInverseKinematics(ik)
    except KServerException as ex:
        print("Unable to compute inverse kinematics")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return False

    actuator_count = base.GetActuatorCount()
    out = [0.0] * actuator_count.count

    idx = 0
    for joint_angle in computed.joint_angles:
        if idx >= actuator_count.count:
            break
        out[idx] = joint_angle.value
        idx += 1

    if idx < actuator_count.count:
        print("[IK] Warning: computed joint list shorter than actuator_count ({} < {}).".format(idx, actuator_count.count))
        return False

    return out


def angular_action_movement(base, movement_type, target):
    action = Base_pb2.Action()
    action.name = "Angular action movement"
    action.application_data = ""

    actuator_count = base.GetActuatorCount()
    ensure_single_level_servoing(base)

    if movement_type == "joint":
        joints = target
    elif movement_type == "pose":
        joints = inverse_kinematics(base, target)
        if joints is False:
            return False
    else:
        print("movement_type invalide:", movement_type)
        return False

    if joints is None or len(joints) < actuator_count.count:
        print("Joints invalides (len={} / attendu={}).".format(0 if joints is None else len(joints), actuator_count.count))
        return False

    for joint_id in range(actuator_count.count):
        ja = action.reach_joint_angles.joint_angles.joint_angles.add()
        ja.joint_identifier = joint_id
        ja.value = float(joints[joint_id])

    e = threading.Event()
    notif_handle = base.OnNotificationActionTopic(check_for_end_or_abort(e), Base_pb2.NotificationOptions())

    print("Executing action")
    base.ExecuteAction(action)

    finished = _wait_event_or_emergency(e, TIMEOUT_DURATION, base, where="angular_action_movement")
    base.Unsubscribe(notif_handle)

    if finished:
        print("Angular movement completed")
    else:
        print("Timeout on action notification wait")
    return finished


def get_current_tool_xyz(base_cyclic):
    fb = base_cyclic.RefreshFeedback()
    return [fb.base.tool_pose_x, fb.base.tool_pose_y, fb.base.tool_pose_z]


def _clamp(v, vmin, vmax):
    if v < vmin:
        return vmin
    if v > vmax:
        return vmax
    return v


def _dist3(a, b):
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    dz = float(a[2]) - float(b[2])
    return (dx*dx + dy*dy + dz*dz) ** 0.5


def approach_hand_twist(base, base_cyclic):
    ensure_single_level_servoing(base)

    t_start = time.time()
    t_ok_start = None
    target_f = None

    print("[TWIST] Demarrage approche main. Enter={:.3f} m / Exit={:.3f} m, maintien={} s".format(
        DIST_ENTER_M, DIST_EXIT_M, HOLD_OK_TIME_S
    ))
    print("[TWIST] Filtre EMA actif. tau={:.3f}s, CTRL_HZ={:.1f}Hz".format(FILTER_TAU_S, CTRL_HZ))

    while True:
        check_emergency_and_abort(base, where="approach_hand_twist")

        now = time.time()

        if (now - t_start) > APPROACH_TIMEOUT_S:
            print("[TWIST] Timeout approche ({:.1f}s). Stop.".format(APPROACH_TIMEOUT_S))
            stop_robot(base)
            return False

        hand_cam = get_hand_coord()
        if hand_cam == [0.0, 0.0, 0.0]:
            stop_robot(base)
            t_ok_start = None
            target_f = None
            time.sleep(0.03)
            continue

        Xk, Yk, Zk = camera3d_coordinates_to_kinova_xyz_m(hand_cam[0], hand_cam[1], hand_cam[2])

        Xk = _clamp(Xk + GIVE_X_NEG_OFFSET_M, MIN_X, MAX_X)
        Yk = _clamp(Yk, MIN_Y, MAX_Y)
        Zk = _clamp(Zk, MIN_Z, MAX_Z)

        alpha = CTRL_DT / (FILTER_TAU_S + CTRL_DT)
        if target_f is None:
            target_f = [Xk, Yk, Zk]
        else:
            target_f[0] = target_f[0] + alpha * (Xk - target_f[0])
            target_f[1] = target_f[1] + alpha * (Yk - target_f[1])
            target_f[2] = target_f[2] + alpha * (Zk - target_f[2])

        tool = get_current_tool_xyz(base_cyclic)
        tool_eff = [tool[0] + TOOL_X_ADD_M, tool[1], tool[2]]
        dist = _dist3(target_f, tool_eff)

        in_zone = (dist <= DIST_ENTER_M) or (t_ok_start is not None and dist <= DIST_EXIT_M)
        if in_zone:
            send_twist_base(base, 0.0, 0.0, 0.0)
            if t_ok_start is None:
                t_ok_start = now

            if (now - t_ok_start) >= HOLD_OK_TIME_S:
                print("[TWIST] OK stable (FREEZE): dist={:.3f} m pendant {:.1f} s. Stop.".format(dist, HOLD_OK_TIME_S))
                stop_twist_hard(base)
                time.sleep(0.15)
                return True

            time.sleep(CTRL_DT)
            continue
        else:
            t_ok_start = None

        ex = target_f[0] - tool_eff[0]
        ey = target_f[1] - tool_eff[1]
        ez = target_f[2] - tool_eff[2]
        vx = _clamp(KP_LIN * ex, -V_MAX, V_MAX)
        vy = _clamp(KP_LIN * ey, -V_MAX, V_MAX)
        vz = _clamp(KP_LIN * ez, -V_MAX, V_MAX)

        if abs(ex) < 0.005:
            vx = 0.0
        if abs(ey) < 0.005:
            vy = 0.0
        if abs(ez) < 0.005:
            vz = 0.0

        send_twist_base(base, vx, vy, vz)

        print("[TWIST] dist={:.3f}  tool=({:.3f},{:.3f},{:.3f})  tool_eff=({:.3f},{:.3f},{:.3f})  target_f=({:.3f},{:.3f},{:.3f})  v=({:.3f},{:.3f},{:.3f})".format(
            dist,
            tool[0], tool[1], tool[2],
            tool_eff[0], tool_eff[1], tool_eff[2],
            target_f[0], target_f[1], target_f[2],
            vx, vy, vz
        ))

        time.sleep(CTRL_DT)


def correction_parabolique(x):
    return ((x-300)**2)/300000


def take_object_sequence(base, router):
    check_emergency_and_abort(base, where="take_object_sequence(start)")

    success = True
    success &= move_to_known_position(base, "transition")
    success &= move_to_known_position(base, "attaque")

    base_cyclic = BaseCyclicClient(router)
    fb = base_cyclic.RefreshFeedback()

    pose_attaque = [
        fb.base.tool_pose_x,
        fb.base.tool_pose_y,
        fb.base.tool_pose_z,
        fb.base.tool_pose_theta_x,
        fb.base.tool_pose_theta_y,
        fb.base.tool_pose_theta_z
    ]

    object_xy = [0.0, 0.0]
    pickup_threshold = 270

    while object_xy[1] < pickup_threshold:
        check_emergency_and_abort(base, where="take_object_sequence(wait_object)")
        object_xy = get_object_coord()
        print("coord x de l'objet : ", object_xy[0], " coord y de l'objet : ", object_xy[1])
        time.sleep(0.02)

    x_obj_m, y_obj_m = camera_pixels_to_kinova_xy_m(object_xy[0], object_xy[1])

    x_offset_obj = 0.0
    y_offset_obj = 0.15 + correction_parabolique(object_xy[0])
    print("y offset : ", y_offset_obj)
    z_offset_obj = -0.055

    take_object_pose = pose_attaque[:]
    take_object_pose[0] = x_obj_m + x_offset_obj
    take_object_pose[1] = y_obj_m + y_offset_obj
    take_object_pose[2] = take_object_pose[2] + z_offset_obj

    check_emergency_and_abort(base, where="take_object_sequence(before_move_pose)")
    success &= angular_action_movement(base, "pose", take_object_pose)
    print("pos prise obj")

    check_emergency_and_abort(base, where="take_object_sequence(before_gripper_close)")
    GripperCommand(base).Send_Gripper_Command_Close()
    print("prise objet")

    check_emergency_and_abort(base, where="take_object_sequence(return)")
    success &= move_to_known_position(base, "attaque")
    success &= move_to_known_position(base, "transition")

    return success


def give_object_sequence(base, base_cyclic):
    check_emergency_and_abort(base, where="give_object_sequence(start)")

    success = True
    success &= move_to_known_position(base, "transition")
    success &= move_to_known_position(base, "give")
    if not success:
        stop_robot(base)
        return False

    check_emergency_and_abort(base, where="give_object_sequence(before_twist)")
    ok = approach_hand_twist(base, base_cyclic)
    if not ok:
        stop_robot(base)
        return False

    check_emergency_and_abort(base, where="give_object_sequence(before_gripper_open)")
    print("[GIVE] Ouverture du gripper.")
    GripperCommand(base).Send_Gripper_Command_Open()
    time.sleep(0.3)

    check_emergency_and_abort(base, where="give_object_sequence(retract)")
    success &= move_to_known_position(base, "give")
    success &= move_to_known_position(base, "transition")
    stop_robot(base)
    return success


# ============================================================
# MAIN
# ============================================================
def main():
    import argparse
    sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
    import utilities

    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["take", "give", "both"], default="both",
                        help="take: prise objet | give: remise main | both: prise puis remise")
    parser.add_argument("--loops", type=int, default=20, help="Nombre de cycles (mode both)")
    args = utilities.parseConnectionArguments(parser)

    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        _self_test_transform()

        start_mqtt_receiver()
        try:
            success = True

            if args.mode == "take":
                GripperCommand(base).Send_Gripper_Command_Open()
                success &= move_to_known_position(base, "attaque")
                success &= take_object_sequence(base, router)

            elif args.mode == "give":
                GripperCommand(base).Send_Gripper_Command_Close()
                success &= move_to_known_position(base, "Home")
                success &= give_object_sequence(base, base_cyclic)

            else:  # both
                GripperCommand(base).Send_Gripper_Command_Open()
                success &= move_to_known_position(base, "transition")

                n = int(args.loops) if args.loops is not None else 1
                if n < 1:
                    n = 1

                for i in range(n):
                    check_emergency_and_abort(base, where="main(loop)")
                    print("")
                    print("========== CYCLE {} / {} ==========".format(i + 1, n))
                    success &= take_object_sequence(base, router)
                    if not success:
                        break
                    success &= give_object_sequence(base, base_cyclic)
                    if not success:
                        break

            return 0 if success else 1

        except StopRequested as e:
            # arrêt d'urgence : stop bras + sortie propre
            print("\n[STOP] {}\n".format(str(e)))
            try:
                publish_kinova_status("emergency")
            except Exception:
                pass
            try:
                stop_twist_hard(base)
            except Exception:
                pass
            return 0

        finally:
            try:
                stop_twist_hard(base)
            except Exception:
                pass
            stop_mqtt_receiver()


if __name__ == "__main__":
    exit(main())
