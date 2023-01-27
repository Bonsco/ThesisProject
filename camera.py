import cv2

class Camera:
    def __init__(self):
        self.video_capture = cv2.VideoCapture(0)

    def read(self):
        return self.video_capture.read()

    def release(self):
        self.video_capture.release()