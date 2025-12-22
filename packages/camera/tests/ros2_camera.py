

from lerobot_camera_ros2.config_ros2_camera import ROS2CameraConfig
from lerobot_camera_ros2.ros2_camera import ROS2Camera
import cv2

config = ROS2CameraConfig(
    topic="/camera/image/compressed",
    is_compressed=True,
    width=640,
    height=480,
    fps=30,
)

camera = ROS2Camera(config)
camera.connect()

while True:
    image = camera.async_read()
    print(image.shape)
    cv2.imshow("Image", image)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

camera.disconnect()
cv2.destroyAllWindows()
