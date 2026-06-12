"""
YOLO detector pour le Spot.

Activation a la demande via /yolo_enable (Bool publie par perception_supervisor).
- enable=False : les frames sont droppees sans inference (CPU quasi nul)
- enable=True  : inference sur 1 frame sur N (decimation pour ~10 Hz)

Souscrit a /camera/color/image_raw/compressed (JPEG, BEST_EFFORT) pour
soulager le bus DDS local. Decompression via cv2.imdecode.

Publie /yolo_detections (Detection2DArray) avec ByteTrack pour filtrer les
faux positifs d'une frame.
"""

import numpy as np
import cv2
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Header
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose

import supervision as sv


# ----------- Configuration --------------
MODEL_PATH = '/opt/yolo_ws/src/yolo_detector/models/Yolov8m_SPOT.onnx'
IMAGE_TOPIC = '/camera/color/image_raw/compressed'
ENABLE_TOPIC = '/yolo_enable'
DETECTIONS_TOPIC = '/yolo_detections'

# Hyperparametres detection
CONF_THRESHOLD = 0.7
IOU_THRESHOLD = 0.1
MAX_DETECTIONS = 1
INPUT_SIZE = 320

# Decimation : on traite 1 frame sur N (cam a 30 Hz -> yolo a 30/N Hz)
FRAME_SKIP = 6   # 30 / 6 = 5 Hz

# Mode demarrage : par defaut yolo est inactif, le supervisor doit l'activer
START_ENABLED = False


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        # Chargement modele ONNX
        self.get_logger().info(f"Chargement du modele : {MODEL_PATH}")
        self.session = ort.InferenceSession(
            MODEL_PATH, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
        self.get_logger().info(
            f"Modele charge. Input '{self.input_name}', taille {INPUT_SIZE}x{INPUT_SIZE}, "
            f"conf={CONF_THRESHOLD}, iou={IOU_THRESHOLD}, max_det={MAX_DETECTIONS}, "
            f"frame_skip={FRAME_SKIP} (cible {30/FRAME_SKIP:.0f} Hz)")

        # Tracker ByteTrack
        self.tracker = sv.ByteTrack()

        # Etat : activation + compteur de frames
        self.enabled = START_ENABLED
        self.frame_counter = 0

        # QoS BEST_EFFORT pour la cam compressee
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.create_subscription(
            CompressedImage, IMAGE_TOPIC, self.on_image, cam_qos)

        # Subscription au signal d'activation (RELIABLE car peu de messages)
        self.create_subscription(
            Bool, ENABLE_TOPIC, self.on_enable, 10)

        self.pub_detections = self.create_publisher(
            Detection2DArray, DETECTIONS_TOPIC, 10)

        self.get_logger().info(
            f"YOLO detector pret. Etat initial : "
            f"{'ENABLED' if self.enabled else 'DISABLED (en attente de /yolo_enable)'}.")

    def on_enable(self, msg: Bool):
        """Activation/desactivation a la demande du supervisor."""
        new_state = bool(msg.data)
        if new_state != self.enabled:
            self.enabled = new_state
            self.frame_counter = 0
            if self.enabled:
                # Reset du tracker quand on reactive
                self.tracker = sv.ByteTrack()
                self.get_logger().info("YOLO ACTIVE (tracker reinitialise).")
            else:
                self.get_logger().info("YOLO DESACTIVE.")

    def on_image(self, msg: CompressedImage):
        # Si desactive : on drop sans rien faire (CPU quasi nul)
        if not self.enabled:
            return

        # Decimation : on traite 1 frame sur FRAME_SKIP
        self.frame_counter += 1
        if self.frame_counter % FRAME_SKIP != 0:
            return

        # Decompression JPEG -> OpenCV BGR
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_img is None:
                self.get_logger().warn("Decode JPEG echoue",
                                       throttle_duration_sec=2.0)
                return
        except Exception as e:
            self.get_logger().warn(f"Erreur decompression : {e}",
                                   throttle_duration_sec=2.0)
            return

        h_orig, w_orig = cv_img.shape[:2]

        # Preprocessing : resize 320x320 + normalisation + format ONNX
        resized = cv2.resize(cv_img, (INPUT_SIZE, INPUT_SIZE))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        normalized = rgb.astype(np.float32) / 255.0
        chw = normalized.transpose(2, 0, 1)
        input_tensor = np.expand_dims(chw, axis=0)

        # Inference
        outputs = self.session.run(None, {self.input_name: input_tensor})
        predictions = outputs[0][0].T  # (2100, 5)

        # Filtrage par confiance
        confidences = predictions[:, 4]
        mask = confidences > CONF_THRESHOLD
        predictions = predictions[mask]

        if len(predictions) == 0:
            self._publish_empty(msg.header)
            return

        # Conversion boxes [cx, cy, w, h] -> [x1, y1, x2, y2]
        boxes_input = predictions[:, :4]
        scores = predictions[:, 4]

        x1 = boxes_input[:, 0] - boxes_input[:, 2] / 2
        y1 = boxes_input[:, 1] - boxes_input[:, 3] / 2
        x2 = boxes_input[:, 0] + boxes_input[:, 2] / 2
        y2 = boxes_input[:, 1] + boxes_input[:, 3] / 2
        boxes_xyxy = np.stack([x1, y1, x2, y2], axis=1)

        # Mise a l'echelle vers la resolution originale
        scale_x = w_orig / INPUT_SIZE
        scale_y = h_orig / INPUT_SIZE
        boxes_xyxy_orig = boxes_xyxy * np.array([scale_x, scale_y, scale_x, scale_y])

        # NMS via OpenCV
        boxes_nms = np.stack([
            boxes_xyxy_orig[:, 0],
            boxes_xyxy_orig[:, 1],
            boxes_xyxy_orig[:, 2] - boxes_xyxy_orig[:, 0],
            boxes_xyxy_orig[:, 3] - boxes_xyxy_orig[:, 1]
        ], axis=1)

        indices = cv2.dnn.NMSBoxes(
            boxes_nms.tolist(), scores.tolist(),
            CONF_THRESHOLD, IOU_THRESHOLD)

        if len(indices) == 0:
            self._publish_empty(msg.header)
            return

        indices = indices.flatten()[:MAX_DETECTIONS]

        kept_boxes = boxes_xyxy_orig[indices]
        kept_scores = scores[indices]
        kept_class_ids = np.zeros(len(indices), dtype=int)

        detections = sv.Detections(
            xyxy=kept_boxes,
            confidence=kept_scores,
            class_id=kept_class_ids)

        tracked = self.tracker.update_with_detections(detections)

        if len(tracked) == 0:
            self._publish_empty(msg.header)
            return

        # Construction message de sortie
        out = Detection2DArray()
        out.header = msg.header
        for i in range(len(tracked)):
            box = tracked.xyxy[i]
            score = float(tracked.confidence[i])
            tid = int(tracked.tracker_id[i])

            det = Detection2D()
            det.bbox = BoundingBox2D()
            det.bbox.center.x = float((box[0] + box[2]) / 2)
            det.bbox.center.y = float((box[1] + box[3]) / 2)
            det.bbox.size_x = float(box[2] - box[0])
            det.bbox.size_y = float(box[3] - box[1])

            hyp = ObjectHypothesisWithPose()
            hyp.id = f"spot_{tid}"
            hyp.score = score
            det.results.append(hyp)

            out.detections.append(det)

            self.get_logger().info(
                f"Spot tracke (id={tid}, score={score:.2f}) "
                f"bbox ({int(det.bbox.center.x)}, {int(det.bbox.center.y)})")

        self.pub_detections.publish(out)

    def _publish_empty(self, header: Header):
        out = Detection2DArray()
        out.header = header
        self.pub_detections.publish(out)


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
