import cv2
import numpy as np

def show_image(image: np.ndarray, title: str = "Image", wait_key: bool = True):
    if image.dtype == bool:
        image = image.astype(np.uint8) * 255
    cv2.imshow(title, image)
    if wait_key:
        while True:
            key = cv2.waitKey(0)
            if key == 32:  # spacebar
                break
        cv2.destroyWindow(title)