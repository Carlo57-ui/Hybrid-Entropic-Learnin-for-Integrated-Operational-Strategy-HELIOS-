# obstacle_detector.py

import depthai as dai
import cv2
import numpy as np

class ObstacleDetector:
    def __init__(self, max_distance=2000, timeout_ms=5000):
        """
        max_distance: distancia máxima de obstáculos en mm
        timeout_ms: tiempo máximo para esperar un frame
        """
        self.max_distance = max_distance
        self.timeout_ms = timeout_ms

        # Pipeline
        self.pipeline = dai.Pipeline()

        # Cámaras mono
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)

        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        stereo.initialConfig.setConfidenceThreshold(200)

        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        # Salida de profundidad
        xoutDepth = self.pipeline.create(dai.node.XLinkOut)
        xoutDepth.setStreamName("depth")
        stereo.depth.link(xoutDepth.input)

        # Inicializar dispositivo
        try:
            self.device = dai.Device(self.pipeline)
            calibData = self.device.readCalibration()
            intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.LEFT, 400, 400)
            self.fx, self.fy = intrinsics[0][0], intrinsics[1][1]
            self.cx, self.cy = intrinsics[0][2], intrinsics[1][2]

            self.depthQueue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        except Exception as e:
            
            #print("Error al inicializar la cámara OAK-D:", e)
            self.device = None

    def get_obstacles(self):
        """
        Retorna lista de obstáculos [(X, Z), (X, Z), ...] en metros.
        Si falla la cámara, devuelve lista vacía.
        """
        if self.device is None:
            return []

        try:
            frame = self.depthQueue.get(timeout=self.timeout_ms)
            depthFrame = frame.getFrame()
        except RuntimeError as e:
            #print("Warning: no se pudo leer frame de profundidad:", e)
            return []
        except Exception as e:
            #print("Error inesperado al leer frame:", e)
            return []

        # Umbral de obstáculos
        mask = (depthFrame > 0) & (depthFrame < self.max_distance)
        mask = mask.astype(np.uint8) * 255

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        obstacles = []
        for cnt in contours:
            if cv2.contourArea(cnt) < 200:  # ignorar ruido pequeño
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            roi = depthFrame[y:y+h, x:x+w]
            roi_nonzero = roi[np.nonzero(roi)]
            if len(roi_nonzero) == 0:
                continue

            Z = np.min(roi_nonzero) / 1000.0  # metros
            u = x + w // 2
            v = y + h // 2
            X = ((u - self.cx) * (Z * 1000)) / self.fx / 1000.0
            obstacles.append((X, Z))

        return obstacles
