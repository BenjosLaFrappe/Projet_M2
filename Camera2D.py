import cv2
from ultralytics import YOLO
import os
import paho.mqtt.client as paho
import time
import threading

# --- MQTT Configuration ---
MQTT_BROKER_IP = "127.0.0.1"
MQTT_PORT = 1883

# IMPORTANT: Kinova utilise ce topic, on ne le change pas
MQTT_COMMAND_TOPIC = "camera/object"

# Nouveau: topic d'état pour l'IHM (LED)
MQTT_STATE_TOPIC = "camera2d/state"

# Emergency stop (IHM)
MQTT_SCADA_CMD_TOPIC = "scada/cmd"
EMERGENCY_PAYLOAD = "emergency"
_stop_event = threading.Event()

# --- NO OBJECT settings ---
NO_OBJECT_MESSAGE = "no_object"
NO_OBJECT_DELAY_S = 0.30  # attends 0.30s sans détection avant d'envoyer "no_object" (anti-clignotement)


def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("Connecté au broker MQTT!")

        # Subscribe emergency stop
        try:
            client.subscribe(MQTT_SCADA_CMD_TOPIC, qos=1)
            print("Abonné à '{}' (Emergency Stop)".format(MQTT_SCADA_CMD_TOPIC))
        except Exception as e:
            print("Impossible de subscribe scada/cmd:", e)

        # LED ON (retained)
        client.publish(MQTT_STATE_TOPIC, "on", qos=1, retain=True)
    else:
        print("Échec de la connexion, code de retour:", rc)


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
            print("[EMERGENCY] Reçu '{}' sur {} -> arrêt Camera2D".format(payload, MQTT_SCADA_CMD_TOPIC))
            _stop_event.set()


def put_text_outline(img, text, org, fontFace, fontScale,
                     color_fg=(255, 255, 255), thickness_fg=2,
                     color_bg=(0, 0, 0), thickness_bg=6,
                     lineType=cv2.LINE_AA):
    cv2.putText(img, text, org, fontFace, fontScale, color_bg, thickness_bg, lineType)
    cv2.putText(img, text, org, fontFace, fontScale, color_fg, thickness_fg, lineType)


def arrowed_line_outline(img, pt1, pt2,
                         color_fg=(255, 255, 255), thickness_fg=2,
                         color_bg=(0, 0, 0), thickness_bg=6,
                         tipLength=0.25):
    cv2.arrowedLine(img, pt1, pt2, color_bg, thickness_bg, cv2.LINE_AA, 0, tipLength)
    cv2.arrowedLine(img, pt1, pt2, color_fg, thickness_fg, cv2.LINE_AA, 0, tipLength)


def draw_xy_axes_top_left(frame, margin=25, length=60):
    """
    Repère en haut à gauche :
      - X vers la droite
      - Y vers le bas (comme image OpenCV)
    """
    ox, oy = margin, margin

    # Axe Y (droite)
    arrowed_line_outline(frame, (ox, oy), (ox + length, oy))
    put_text_outline(frame, "Y", (ox + length + 8, oy + 6),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.6)

    # Axe X (bas)
    arrowed_line_outline(frame, (ox, oy), (ox, oy + length))
    put_text_outline(frame, " X", (ox - 12, oy + length + 22),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.6)


# --- MQTT client ---
client = paho.Client(paho.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message

# IMPORTANT: Last Will = off si le script crash / perd la connexion
client.will_set(MQTT_STATE_TOPIC, "off", qos=1, retain=True)

cap = None

try:
    print("Tentative de connexion au broker MQTT à {}:{}...".format(MQTT_BROKER_IP, MQTT_PORT))
    client.connect(MQTT_BROKER_IP, MQTT_PORT, 60)
    client.loop_start()

    time.sleep(1)

    # Publier le message 'ready' initial (inchangé)
    client.publish(MQTT_COMMAND_TOPIC, "ready")
    print("Message 'ready' publié sur le topic '{}'.".format(MQTT_COMMAND_TOPIC))

    # Charge le modèle (chemin relatif au script)
    script_dir = os.path.dirname(__file__) if "__file__" in locals() else os.getcwd()
    model_path = os.path.join(script_dir, "best.pt")
    model = YOLO(model_path)

    # Ouverture caméra
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        raise IOError("Impossible d'ouvrir la caméra")

    print("Caméra ouverte — Appuie sur Q pour quitter (ou Emergency Stop sur IHM)")

    last_detection_state = None
    last_time_object_seen = time.time()

    while True:
        # Emergency stop check
        if _stop_event.is_set():
            print("[EMERGENCY] Stop demandé -> sortie boucle Camera2D")
            break

        ret, frame = cap.read()
        if not ret:
            break

        draw_xy_axes_top_left(frame)

        results = model(frame, conf=0.25, verbose=False)
        detections = []  # (cx, cy, x1, y1, x2, y2)

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                detections.append((cx, cy, x1, y1, x2, y2))

        if len(detections) > 0:
            last_time_object_seen = time.time()

            best = max(detections, key=lambda d: d[0])  # inchangé: cx max
            cx, cy, x1, y1, x2, y2 = best

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

            cv2.putText(frame, "square", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.putText(frame, "center ({}, {})".format(cy, cx), (x1, y2 + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # MQTT: format inchangé pour Kinova (swap comme avant)
            center_coords_message = "{{'center_x': {}, 'center_y': {}}}".format(cy, cx)
            client.publish(MQTT_COMMAND_TOPIC, center_coords_message)
            print("Objet détecté (cx max) → {}".format(center_coords_message))

        now = time.time()
        no_object_condition = (now - last_time_object_seen) >= NO_OBJECT_DELAY_S

        if no_object_condition:
            if last_detection_state != False:
                client.publish(MQTT_COMMAND_TOPIC, NO_OBJECT_MESSAGE)
                print("Aucun objet détecté → '{}' envoyé".format(NO_OBJECT_MESSAGE))
            last_detection_state = False
        else:
            last_detection_state = True

        cv2.imshow("Detection du carre", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print("Erreur générale:", e)

finally:
    # IMPORTANT: éteint la LED proprement à la fin
    try:
        client.publish(MQTT_STATE_TOPIC, "off", qos=1, retain=True)
        time.sleep(0.1)
    except Exception:
        pass

    if client.is_connected():
        client.loop_stop()
        client.disconnect()
        print("Déconnecté du broker MQTT.")

    if cap is not None and cap.isOpened():
        cap.release()

    cv2.destroyAllWindows()