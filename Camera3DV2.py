import os
os.environ["GLOG_minloglevel"] = "2"

import cv2
import numpy as np
import mediapipe as mp
import pyzed.sl as sl
import paho.mqtt.client as paho
import time
import json
import threading

# ==========================================================
# MQTT CONFIG
# ==========================================================
MQTT_BROKER_IP = "127.0.0.1"
MQTT_PORT = 1883
MQTT_HAND_TOPIC = "camera/hand"

# Emergency stop (IHM)
MQTT_SCADA_CMD_TOPIC = "scada/cmd"
EMERGENCY_PAYLOAD = "emergency"
_stop_event = threading.Event()

# Publie à 20 Hz (coordonnées UNIQUEMENT quand une seule main est DANS la zone)
PUBLISH_HZ = 20
PUBLISH_INTERVAL_S = 1.0 / PUBLISH_HZ

# no_hand quand aucune main détectée (anti faux-positifs au lancement)
NO_HAND_FRAMES_THRESHOLD = 20     # ~0.66s à 30 fps

# Debug
PRINT_PUBLISH = True
PRINT_DEBUG = False

# ==========================================================
# ZONE (SPHERE) OVERLAY (repère caméra MQTT : x droite, y haut, z avant)
# ==========================================================
SPHERE_CENTER_CAM = (0.0, 0.0, 0.55)   # (x,y,z) en mètres
SPHERE_RADIUS_M = 0.30                # rayon en mètres


def cam_xyz_to_zed_xyz(x_cam, y_cam, z_cam):
    """
    Repère MQTT (cam): x=droite, y=haut, z=avant(profondeur)
    ZED RIGHT_HANDED_Z_UP: X=droite, Y=avant, Z=haut
    => (X,Y,Z)zed = (x_cam, z_cam, y_cam)
    """
    return float(x_cam), float(z_cam), float(y_cam)


def project_zed_xyz_to_pixel(X, Y, Z, fx, fy, cx, cy):
    """
    Projection pinhole (image LEFT) :
      u = cx + fx * X/Y
      v = cy - fy * Z/Y
    """
    if Y <= 1e-6:
        return None
    u = int(cx + fx * (X / Y))
    v = int(cy - fy * (Z / Y))
    return (u, v)


def draw_sphere_overlay(img, fx, fy, cx, cy, center_cam, radius_m):
    """
    Dessine la projection 2D de la sphère 3D (cercle).
    r_px ≈ f * R / profondeur
    """
    x_cam, y_cam, z_cam = center_cam
    Xz, Yz, Zz = cam_xyz_to_zed_xyz(x_cam, y_cam, z_cam)

    uv = project_zed_xyz_to_pixel(Xz, Yz, Zz, fx, fy, cx, cy)
    if uv is None:
        return

    f = 0.5 * (fx + fy)
    r_px = int(abs(f * (radius_m / Yz)))
    if r_px <= 1:
        return

    cv2.circle(img, uv, r_px, (255, 255, 0), 2)
    cv2.circle(img, uv, 4, (255, 255, 0), -1)

    text = "Zone de retrait objet"
    org = (max(10, uv[0] - 140), max(25, uv[1] - r_px - 10))
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.7

    cv2.putText(img, text, org, font, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, text, org, font, scale, (255, 255, 0), 2, cv2.LINE_AA)


def is_inside_sphere_cam(x, y, z, center_cam, radius_m):
    dx = float(x) - float(center_cam[0])
    dy = float(y) - float(center_cam[1])
    dz = float(z) - float(center_cam[2])
    return (dx * dx + dy * dy + dz * dz) <= (radius_m * radius_m)


# ==========================================================
# MQTT CALLBACKS
# ==========================================================
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("Connecté au broker MQTT!")

        # Subscribe emergency stop
        try:
            client.subscribe(MQTT_SCADA_CMD_TOPIC, qos=1)
            print("Abonné à '{}' (Emergency Stop)".format(MQTT_SCADA_CMD_TOPIC))
        except Exception as e:
            print("Impossible de subscribe scada/cmd:", e)
    else:
        print(f"Échec connexion MQTT, rc={rc}")


def on_message(client, userdata, msg):
    # Emergency stop handler
    try:
        topic = msg.topic
        payload = msg.payload.decode("utf-8", errors="ignore").strip()
    except Exception:
        return

    if topic == MQTT_SCADA_CMD_TOPIC:
        pl = payload.lower()
        if pl == EMERGENCY_PAYLOAD or "emergency" in pl or pl in ("estop", "e-stop"):
            print("[EMERGENCY] Reçu '{}' sur {} -> arrêt Camera3D".format(payload, MQTT_SCADA_CMD_TOPIC))
            _stop_event.set()


client = paho.Client(paho.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message

# ==========================================================
# INIT ZED
# ==========================================================
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.depth_mode = sl.DEPTH_MODE.ULTRA
init_params.coordinate_units = sl.UNIT.METER
init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP

if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("Erreur : impossible d'ouvrir la ZED")
    raise SystemExit(1)

cam_info = zed.get_camera_information()
left_cam = cam_info.camera_configuration.calibration_parameters.left_cam
FX = float(left_cam.fx)
FY = float(left_cam.fy)
CX = float(left_cam.cx)
CY = float(left_cam.cy)

rtime = sl.RuntimeParameters()
image_zed = sl.Mat()
point_cloud = sl.Mat()

# ==========================================================
# MEDIAPIPE HANDS (multi-hands)
# ==========================================================
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=4,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)
mp_draw = mp.solutions.drawing_utils


# ==========================================================
# DEPTH INTERPOLATION
# ==========================================================
def get_depth_nearby(point_cloud, u, v, w, h, window=4):
    valid = []
    for i in range(-window, window + 1):
        for j in range(-window, window + 1):
            uu = max(0, min(u + i, w - 1))
            vv = max(0, min(v + j, h - 1))

            err, p = point_cloud.get_value(uu, vv)
            if err == sl.ERROR_CODE.SUCCESS:
                xyz = np.array(p[:3], dtype=np.float32)
                if np.all(np.isfinite(xyz)):
                    valid.append(xyz)

    if not valid:
        return None
    return np.mean(valid, axis=0)


# ==========================================================
# MQTT CONNECT
# ==========================================================
client.connect(MQTT_BROKER_IP, MQTT_PORT, 60)
client.loop_start()
time.sleep(0.3)
client.publish(MQTT_HAND_TOPIC, "ready")
print("Programme lancé - Appuie sur Q pour quitter (ou Emergency Stop sur IHM)")

# ==========================================================
# STATE
# ==========================================================
last_coords_publish_time = 0.0
no_hand_sent = False
no_hand_frames = 0


try:
    while True:
        # Emergency stop check
        if _stop_event.is_set():
            print("[EMERGENCY] Stop demandé -> sortie boucle Camera3D")
            break

        if zed.grab(rtime) != sl.ERROR_CODE.SUCCESS:
            continue

        zed.retrieve_image(image_zed, sl.VIEW.LEFT)
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)

        img = image_zed.get_data()[:, :, :3].copy()
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_display = cv2.convertScaleAbs(img)

        draw_sphere_overlay(img_display, FX, FY, CX, CY, SPHERE_CENTER_CAM, SPHERE_RADIUS_M)

        results = hands.process(img_rgb)
        now = time.time()

        any_hand_2d = bool(results.multi_hand_landmarks)
        inside_hands = []
        show_multi_warning = False

        if results.multi_hand_landmarks:
            h, w, _ = img.shape
            palm_ids = [0, 5, 9, 13, 17]

            for hand_landmarks in results.multi_hand_landmarks:
                u_sum, v_sum = 0, 0
                for idx in palm_ids:
                    lm = hand_landmarks.landmark[idx]
                    u_sum += max(0, min(int(lm.x * w), w - 1))
                    v_sum += max(0, min(int(lm.y * h), h - 1))
                palm_u = int(u_sum / len(palm_ids))
                palm_v = int(v_sum / len(palm_ids))

                pts_3d = []
                for idx in palm_ids:
                    lm = hand_landmarks.landmark[idx]
                    u = max(0, min(int(lm.x * w), w - 1))
                    v = max(0, min(int(lm.y * h), h - 1))
                    p = get_depth_nearby(point_cloud, u, v, w, h, window=4)
                    if p is not None:
                        pts_3d.append(p)

                if pts_3d:
                    X, Y, Z = np.mean(pts_3d, axis=0)

                    X_cam = float(X)
                    Y_cam = float(Z)
                    Z_cam = float(Y)

                    inside = is_inside_sphere_cam(X_cam, Y_cam, Z_cam, SPHERE_CENTER_CAM, SPHERE_RADIUS_M)
                    palm_color = (255, 0, 0) if inside else (0, 0, 255)

                    cv2.circle(img_display, (palm_u, palm_v), 12, palm_color, -1)
                    cv2.line(img_display, (palm_u - 10, palm_v), (palm_u + 10, palm_v), palm_color, 2)
                    cv2.line(img_display, (palm_u, palm_v - 10), (palm_u, palm_v + 10), palm_color, 2)
                    cv2.putText(
                        img_display,
                        "IN" if inside else "OUT",
                        (palm_u + 15, palm_v + 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        palm_color,
                        2
                    )

                    text_y = max(palm_v - 25, 30)
                    cv2.putText(
                        img_display,
                        f"X:{X_cam:.2f} Y:{Y_cam:.2f} Z:{Z_cam:.2f}",
                        (palm_u, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2
                    )

                    if inside:
                        inside_hands.append({"x": X_cam, "y": Y_cam, "z": Z_cam, "u": palm_u, "v": palm_v})
                else:
                    palm_color = (0, 0, 255)
                    cv2.circle(img_display, (palm_u, palm_v), 12, palm_color, -1)
                    cv2.line(img_display, (palm_u - 10, palm_v), (palm_u + 10, palm_v), palm_color, 2)
                    cv2.line(img_display, (palm_u, palm_v - 10), (palm_u, palm_v + 10), palm_color, 2)
                    cv2.putText(img_display, "OUT", (palm_u + 15, palm_v + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, palm_color, 2)
                    if PRINT_DEBUG:
                        print("[DEBUG] Hand 2D OK mais pas de points 3D valides")

                mp_draw.draw_landmarks(img_display, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        if len(inside_hands) == 1:
            no_hand_sent = False
            no_hand_frames = 0

            if (now - last_coords_publish_time) >= PUBLISH_INTERVAL_S:
                h0 = inside_hands[0]
                payload = json.dumps({
                    "x": round(h0["x"], 4),
                    "y": round(h0["y"], 4),
                    "z": round(h0["z"], 4)
                })
                client.publish(MQTT_HAND_TOPIC, payload)
                last_coords_publish_time = now
                if PRINT_PUBLISH:
                    print(f"MQTT XYZ (1 hand IN zone) -> {payload}")

        else:
            if len(inside_hands) > 1:
                show_multi_warning = True

            if any_hand_2d or (len(inside_hands) > 0):
                no_hand_frames = 0
                if not no_hand_sent:
                    client.publish(MQTT_HAND_TOPIC, "no_hand")
                    no_hand_sent = True
                    if PRINT_PUBLISH:
                        if show_multi_warning:
                            print("MQTT -> no_hand (multiple hands IN zone)")
                        else:
                            print("MQTT -> no_hand (hands present but none IN zone)")
            else:
                no_hand_frames += 1
                if (no_hand_frames >= NO_HAND_FRAMES_THRESHOLD) and (not no_hand_sent):
                    client.publish(MQTT_HAND_TOPIC, "no_hand")
                    no_hand_sent = True
                    if PRINT_PUBLISH:
                        print("MQTT -> no_hand (no hand detected)")

        if show_multi_warning:
            lines = [
                "Plusieurs mains dans la zone de retrait objet,",
                "veuillez n'en laisser qu'une."
            ]
            x, y = 350, 300
            dy = 28
            for i, line in enumerate(lines):
                cv2.putText(img_display, line, (x, y + i * dy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("ZED 2 - Hand 3D Tracking", img_display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # (optionnel) : au stop d'urgence, on peut publier no_hand une dernière fois
    try:
        client.publish(MQTT_HAND_TOPIC, "no_hand")
        time.sleep(0.05)
    except Exception:
        pass

    client.loop_stop()
    client.disconnect()
    hands.close()
    zed.close()
    cv2.destroyAllWindows()
