import cv2
import depthai as dai
import numpy as np
from pathlib import Path
from datetime import datetime

DEVICE_IP = "169.254.1.222"
SAVE_DIR = Path("captures")
SAVE_DIR.mkdir(parents=True, exist_ok=True)

device = dai.Device(dai.DeviceInfo(DEVICE_IP))

with dai.Pipeline(device) as pipeline:
    # RGB camera
    rgb_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    rgb_out = rgb_cam.requestOutput(
        size=(640, 360),
        type=dai.ImgFrame.Type.BGR888p,
        fps=15,
    )
    q_rgb = rgb_out.createOutputQueue()

    # Thermal camera
    thermal = pipeline.create(dai.node.Thermal)
    q_thermal_vis = thermal.color.createOutputQueue()

    pipeline.start()

    while pipeline.isRunning():
        rgb = q_rgb.get().getCvFrame()
        thermal_vis = q_thermal_vis.get().getCvFrame()

        thermal_vis_resized = cv2.resize(
            thermal_vis, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST
        )

        if len(thermal_vis_resized.shape) == 2:
            thermal_vis_resized = cv2.cvtColor(thermal_vis_resized, cv2.COLOR_GRAY2BGR)

        combined = np.hstack((rgb, thermal_vis_resized))
        cv2.imshow("RGB | THERMAL", combined)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        elif key == ord("s"):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

            rgb_path = SAVE_DIR / f"{timestamp}_rgb.png"
            thermal_path = SAVE_DIR / f"{timestamp}_thermal.png"

            cv2.imwrite(str(rgb_path), rgb)
            cv2.imwrite(str(thermal_path), thermal_vis)

            print(f"Saved:\n  {rgb_path}\n  {thermal_path}")

cv2.destroyAllWindows()