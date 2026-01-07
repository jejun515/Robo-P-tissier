#!/usr/bin/env python3
import time
from typing import Optional, Tuple

import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from od_msg.srv import SrvDepthPosition
from pick_and_place_text.realsense import ImgNode
from pick_and_place_text.yolo import YoloModel


class ObjectDetectionNode(Node):
    """
    Service: /get_3d_position (SrvDepthPosition)
    Request:  target (string)
    Response: depth_position (float[3])  -> (X, Y, Z) in CAMERA frame

    - YOLO로 target을 탐지하고
    - bbox 중심 픽셀 (cx, cy)의 depth를 읽어서
    - intrinsics로 카메라 좌표계 (X,Y,Z)로 변환해 반환
    - debug_view=True면 bbox를 OpenCV 창으로 표시
    """

    def __init__(self, model_name: str = "yolo"):
        super().__init__("object_detection_node")

        # ---- options ----
        self.debug_view = True               # bbox 창 띄우기
        self.window_name = "detections"      # imshow window name
        self.depth_scale = None              # None이면 자동 추정 (uint16 -> mm로 가정)
        self.min_depth = 1.0                 # 너무 가까운 값 필터 (단위: depth 단위)
        self.max_depth = 3000.0              # 너무 먼 값 필터 (mm 기준이면 3000mm=3m)

        # ---- camera + model ----
        self.img_node = ImgNode()
        self.model = self._load_model(model_name)

        # intrinsics 확보
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )

        # service 생성 (상대 이름 'get_3d_position' -> 보통 /get_3d_position로 뜸)
        self.create_service(
            SrvDepthPosition,
            "get_3d_position",
            self.handle_get_depth,
        )

        self.get_logger().info("ObjectDetectionNode initialized. Service: /get_3d_position")

    def destroy_node(self):
        # OpenCV window 정리
        try:
            if self.debug_view:
                cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()

    # ---------------------------
    # Model / Service
    # ---------------------------
    def _load_model(self, name: str):
        if name.lower() == "yolo":
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")

    def handle_get_depth(self, request: SrvDepthPosition.Request, response: SrvDepthPosition.Response):
        """클라이언트 요청을 처리해 3D 좌표를 반환."""
        target = request.target
        self.get_logger().info(f"Received request target='{target}'")

        coords = self._compute_position(target)  # (X,Y,Z) in camera frame
        response.depth_position = [float(coords[0]), float(coords[1]), float(coords[2])]
        return response

    # ---------------------------
    # Core pipeline
    # ---------------------------
    def _compute_position(self, target: str) -> Tuple[float, float, float]:
        """
        1) 컬러 프레임/깊이 프레임 최신화
        2) YOLO로 target 검출 -> best bbox
        3) bbox center depth 읽기
        4) pixel -> camera coords 변환
        5) debug: bbox 시각화
        """
        # ImgNode는 별도 Node라서 한번씩 spin해줘야 콜백이 돈다
        rclpy.spin_once(self.img_node, timeout_sec=0.1)

        # 컬러 프레임 확보 (표시/검출에 필요)
        color = self._wait_for_valid_data(self.img_node.get_color_frame, "color frame")

        # YOLO: best detection
        box, score = self.model.get_best_detection(self.img_node, target)
        if box is None or score is None:
            self.get_logger().warn("No detection found.")
            if self.debug_view:
                self._show_debug(color, None, target, None)
            return 0.0, 0.0, 0.0

        # bbox center
        x1, y1, x2, y2 = box
        cx = int((x1 + x2) / 2.0)
        cy = int((y1 + y2) / 2.0)

        # depth read
        cz = self._get_depth(cx, cy)
        if cz is None:
            self.get_logger().warn("Depth out of range or invalid.")
            if self.debug_view:
                self._show_debug(color, box, target, score, center=(cx, cy), depth=None)
            return 0.0, 0.0, 0.0

        # depth scale 자동 추정 (uint16 depth는 보통 mm)
        cz = self._apply_depth_scale(cz)

        # depth sanity check
        if cz < self.min_depth or cz > self.max_depth:
            self.get_logger().warn(f"Depth invalid after scaling: z={cz}")
            if self.debug_view:
                self._show_debug(color, box, target, score, center=(cx, cy), depth=cz)
            return 0.0, 0.0, 0.0

        # pixel -> camera coords
        X, Y, Z = self._pixel_to_camera_coords(cx, cy, cz)

        self.get_logger().info(f"Detection target='{target}' score={score:.3f} "
                               f"pixel=({cx},{cy}) depth={cz} -> cam=({X:.2f},{Y:.2f},{Z:.2f})")

        # debug view
        if self.debug_view:
            self._show_debug(color, box, target, score, center=(cx, cy), depth=cz)

        return float(X), float(Y), float(Z)

    def _get_depth(self, x: int, y: int) -> Optional[float]:
        """픽셀 좌표의 depth 값을 안전하게 읽어옵니다."""
        depth = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        h, w = depth.shape[:2]
        if x < 0 or x >= w or y < 0 or y >= h:
            self.get_logger().warn(f"Coordinates ({x},{y}) out of range: depth size=({w},{h})")
            return None

        z = depth[y, x]

        # NaN/0 체크
        try:
            z_val = float(z)
        except Exception:
            return None

        if z_val == 0.0 or np.isnan(z_val):
            # 주변 픽셀에서 대체값 찾기 (3x3 median)
            z_val = self._median_depth(depth, x, y, k=3)
            if z_val is None:
                return None

        return z_val

    def _median_depth(self, depth: np.ndarray, x: int, y: int, k: int = 3) -> Optional[float]:
        """중심 픽셀 depth가 0/NaN이면 주변에서 median으로 대체."""
        r = k // 2
        h, w = depth.shape[:2]
        xs = max(0, x - r)
        xe = min(w, x + r + 1)
        ys = max(0, y - r)
        ye = min(h, y + r + 1)

        patch = depth[ys:ye, xs:xe].astype(np.float32)
        patch = patch[~np.isnan(patch)]
        patch = patch[patch > 0]

        if patch.size == 0:
            return None
        return float(np.median(patch))

    def _apply_depth_scale(self, z: float) -> float:
        """
        depth_scale이 None이면 dtype 기반으로 자동 추정:
        - uint16이면 보통 mm라고 가정하고 그대로 사용
        - float32/float64면 보통 meter일 가능성이 있는데,
          여기서는 로봇이 mm(posx) 쓰는 경우가 많아서 "m -> mm"로 바꾸고 싶으면
          self.depth_scale = 1000.0 으로 고정하면 됨.
        """
        if self.depth_scale is not None:
            return float(z) * float(self.depth_scale)

        # 자동 추정: depth frame dtype 확인
        depth = self.img_node.get_depth_frame()
        if depth is None:
            return float(z)

        if depth.dtype == np.uint16:
            # RealSense aligned depth에서 흔한 형식: mm
            return float(z)
        else:
            # float depth면 meter일 수도 있음.
            # 기본은 그대로 두되, 필요하면 depth_scale=1000으로 세팅해.
            return float(z)

    def _pixel_to_camera_coords(self, x: int, y: int, z: float) -> Tuple[float, float, float]:
        """픽셀 좌표와 intrinsics를 이용해 카메라 좌표계로 변환합니다."""
        fx = self.intrinsics["fx"]
        fy = self.intrinsics["fy"]
        ppx = self.intrinsics["ppx"]
        ppy = self.intrinsics["ppy"]

        X = (x - ppx) * z / fx
        Y = (y - ppy) * z / fy
        Z = z
        return X, Y, Z

    # ---------------------------
    # Utils
    # ---------------------------
    def _wait_for_valid_data(self, getter, description: str):
        """getter 함수가 유효한 데이터를 반환할 때까지 spin 하며 재시도."""
        data = getter()
        tries = 0
        while data is None or (isinstance(data, np.ndarray) and data.size > 0 and not data.any()):
            tries += 1
            rclpy.spin_once(self.img_node, timeout_sec=0.1)
            if tries % 10 == 0:
                self.get_logger().info(f"Retry getting {description}... ({tries})")
            data = getter()
        return data

    def _show_debug(
        self,
        frame: np.ndarray,
        box,
        target: str,
        score: Optional[float],
        center: Optional[Tuple[int, int]] = None,
        depth: Optional[float] = None,
    ):
        """OpenCV로 bbox/center/label 표시."""
        if frame is None:
            return

        vis = frame.copy()

        if box is not None:
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if center is not None:
            cx, cy = center
            cv2.circle(vis, (int(cx), int(cy)), 4, (0, 0, 255), -1)

        text = target
        if score is not None:
            text += f" {score:.2f}"
        if depth is not None:
            text += f" z={depth:.1f}"

        cv2.putText(
            vis,
            text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.imshow(self.window_name, vis)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode(model_name="yolo")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
