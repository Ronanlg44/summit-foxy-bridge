"""
Detection YOLO du Spot via ONNX Runtime, avec filtrage temporel ByteTrack.

Souscrit a /camera/color/image_raw, fait l'inference YOLOv8m (1 classe = "Spot"),
applique NMS puis ByteTrack pour eliminer les detections d'une seule frame
(faux positifs isoles). Publie /yolo_detections au format vision_msgs avec
le tracker_id dans le champ id de l'hypothesis.

Configuration validee experimentalement sur les sequences DARPA :
  - conf_threshold = 0.7 (peut etre baissée a 0.5 en environnement difficile)
  - iou_threshold  = 0.1 (empeche fusion bbox avec humains)
  - max_det        = 1   (un seul Spot dans le champ)
  - tracker ByteTrack actif (rejette les detections d'une seule frame)
"""

import os
import numpy as np
import cv2
import supervision as sv

import rclpy
from rclpy.node import Node
import onnxruntime as ort

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge


DEFAULT_CONF_THRESHOLD = 0.7
DEFAULT_IOU_THRESHOLD = 0.1
DEFAULT_MAX_DET = 1
DEFAULT_INPUT_SIZE = 320

IMAGE_TOPIC = '/camera/color/image_raw'
DETECTIONS_TOPIC = '/yolo_detections'


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        # Parametres ROS
        self.declare_parameter('model_path', '/opt/yolo_models/Yolov8m_SPOT.onnx')
        self.declare_parameter('conf_threshold', DEFAULT_CONF_THRESHOLD)
        self.declare_parameter('iou_threshold', DEFAULT_IOU_THRESHOLD)
        self.declare_parameter('max_det', DEFAULT_MAX_DET)
        self.declare_parameter('input_size', DEFAULT_INPUT_SIZE)

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.max_det = self.get_parameter('max_det').value
        self.input_size = self.get_parameter('input_size').value

        if not os.path.exists(model_path):
            self.get_logger().error(f"Modele introuvable : {model_path}")
            raise FileNotFoundError(model_path)

        self.get_logger().info(f"Chargement du modele : {model_path}")
        self.session = ort.InferenceSession(
            model_path,
            providers=['CPUExecutionProvider']
        )
        self.input_name = self.session.get_inputs()[0].name
        self.get_logger().info(
            f"Modele charge. Input '{self.input_name}', taille {self.input_size}x{self.input_size}, "
            f"conf={self.conf_threshold}, iou={self.iou_threshold}, max_det={self.max_det}")

        # Tracker ByteTrack : un objet doit etre vu plusieurs frames consecutives
        # avant d'avoir un tracker_id stable (filtre les faux positifs ponctuels)
        self.tracker = sv.ByteTrack()
        self.get_logger().info("Tracker ByteTrack initialise.")

        self.bridge = CvBridge()

        self.create_subscription(Image, IMAGE_TOPIC, self.on_image, 10)
        self.pub_dets = self.create_publisher(Detection2DArray, DETECTIONS_TOPIC, 10)

        self.get_logger().info(
            f"YOLO detector pret. Souscrit a {IMAGE_TOPIC}, publie {DETECTIONS_TOPIC}.")

    def on_image(self, msg: Image):
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Echec conversion image : {e}", throttle_duration_sec=2.0)
            return

        orig_h, orig_w = frame_bgr.shape[:2]

        # Preprocessing
        img = cv2.resize(frame_bgr, (self.input_size, self.input_size))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)

        # Inference
        outputs = self.session.run(None, {self.input_name: img})
        predictions = outputs[0]  # (1, 5, 2100)

        # Parsing YOLOv8 : (1, 5, N) -> (N, 5)
        predictions = predictions[0].T
        scores = predictions[:, 4]
        valid = scores > self.conf_threshold
        predictions = predictions[valid]

        if len(predictions) == 0:
            # Toujours mettre a jour le tracker avec un set vide
            # pour qu'il vieillisse les tracks (et les ferme si pas de detection prolongee)
            empty_detections = sv.Detections.empty()
            self.tracker.update_with_detections(empty_detections)
            self._publish_empty(msg.header)
            return

        # Conversion bbox (cx, cy, w, h) -> (x1, y1, x2, y2)
        boxes = predictions[:, :4].copy()
        boxes_xyxy = np.zeros_like(boxes)
        boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
        boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
        boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
        boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2
        scores_filtered = predictions[:, 4]

        # NMS (avant le tracker, pour limiter le nombre de candidats)
        indices = cv2.dnn.NMSBoxes(
            boxes_xyxy.tolist(),
            scores_filtered.tolist(),
            self.conf_threshold,
            self.iou_threshold,
        )

        if len(indices) == 0:
            empty_detections = sv.Detections.empty()
            self.tracker.update_with_detections(empty_detections)
            self._publish_empty(msg.header)
            return

        if hasattr(indices, 'flatten'):
            indices = indices.flatten()
        indices = sorted(indices, key=lambda i: -scores_filtered[i])
        indices = indices[:self.max_det]

        # Construire un objet sv.Detections pour le tracker
        # (xyxy dans l'image 320x320, on rescale apres)
        kept_xyxy = boxes_xyxy[indices]
        kept_scores = scores_filtered[indices]
        kept_class_ids = np.zeros(len(indices), dtype=int)  # 1 seule classe = 0

        detections = sv.Detections(
            xyxy=kept_xyxy,
            confidence=kept_scores,
            class_id=kept_class_ids,
        )

        # Update tracker : retourne uniquement les detections avec tracker_id stable
        tracked = self.tracker.update_with_detections(detections)

        if len(tracked) == 0:
            # Detection presente mais pas encore de track confirme
            # (typique des premieres frames ou des faux positifs isoles)
            self._publish_empty(msg.header)
            return

        # Rescale vers la resolution image originale
        scale_x = orig_w / self.input_size
        scale_y = orig_h / self.input_size

        det_array = Detection2DArray()
        det_array.header = msg.header

        for i in range(len(tracked)):
            x1, y1, x2, y2 = tracked.xyxy[i]
            score = float(tracked.confidence[i])
            tracker_id = int(tracked.tracker_id[i])

            cx = (x1 + x2) / 2 * scale_x
            cy = (y1 + y2) / 2 * scale_y
            w = (x2 - x1) * scale_x
            h = (y2 - y1) * scale_y

            det = Detection2D()
            det.header = msg.header

            det.bbox = BoundingBox2D()
            det.bbox.center.x = float(cx)
            det.bbox.center.y = float(cy)
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)

            hyp = ObjectHypothesisWithPose()
            hyp.id = f'spot_{tracker_id}'   # ID stable du track
            hyp.score = score
            det.results.append(hyp)

            det_array.detections.append(det)

        self.pub_dets.publish(det_array)
        self.get_logger().info(
            f"Spot tracke (id={tracked.tracker_id[0]}, score={tracked.confidence[0]:.2f}) "
            f"bbox ({(tracked.xyxy[0][0]+tracked.xyxy[0][2])/2*scale_x:.0f}, "
            f"{(tracked.xyxy[0][1]+tracked.xyxy[0][3])/2*scale_y:.0f})",
            throttle_duration_sec=1.0)

    def _publish_empty(self, header):
        msg = Detection2DArray()
        msg.header = header
        self.pub_dets.publish(msg)


def main():
    rclpy.init()
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
