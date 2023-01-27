import cv2
from face_vector import FaceVector
from window import Window

cascPath = "/tracking/haarcascade_frontalface_alt.xml"
faceCascade = cv2.CascadeClassifier(cascPath)

def track(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),
        flags = cv2.CASCADE_SCALE_IMAGE
    )
    
    if len(faces) > 0:
        x, y, w, h = faces[0]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        vector = FaceVector((x + int(w/2), y + int(h/2)), (int(frame.shape[1]/2), int(frame.shape[0]/2)))
        frame = Window.draw_overlay(frame, vector)
        cv2.imshow('Video', frame)
        return vector
    else:
        cv2.imshow('Video', frame)
        return None
    

    
    