import cv2

class Window:
    def set_frame(self, frame):
        self.frame = frame

    @staticmethod
    def draw_overlay(frame, face_vector):

        # Draw center circle
        cv2.circle(frame, (int(frame.shape[1]/2), int(frame.shape[0]/2)), 1, (255,0,0), 5)

        # Draw Blue dot on face
        cv2.circle(frame, face_vector.origin, 1, (0,255,0), 5)

        cv2.arrowedLine(frame, face_vector.origin, face_vector.to, (0,0,255), 2)

        return frame

    
    def show_frame(self):
        cv2.imshow('Video', self.frame)


























    # def draw_overlay(self, face):
    #     if(self.frame == None):
    #         print("Window does not have a frame, call window.set_frame() before using this function.")

    #     cv2.circle(self.frame, (int(self.frame.shape[1]/2), int(self.frame.shape[0]/2)), 1, (255,0,0), 5)

    #     # Display the results
    #     for (top, right, bottom, left), name in zip(face.face_locations, face.face_names):
    #         # Scale back up face locations since the frame we detected in was scaled to 1/4 size
    #         top *= 4
    #         right *= 4
    #         bottom *= 4
    #         left *= 4

    #         width = right - left
    #         height = bottom - top

    #         # Blue dot on face
    #         cv2.circle(self.frame, (int(left+0.5*width), int(top+0.5*height)), 1, (0,255,0), 5)

    #         cv2.arrowedLine(self.frame, (int(left+0.5*width), int(top+0.5*height)), (int(self.frame.shape[1]/2), int(self.frame.shape[0]/2)), (0,0,255), 2)

    #         # # Draw a box around the face
    #         # cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

    #         # # Draw a label with a name below the face
    #         # cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
    #         font = cv2.FONT_HERSHEY_DUPLEX
    #         cv2.putText(self.frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)