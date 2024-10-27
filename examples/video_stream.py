import os
import sys
import cv2
import torch

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from trash_classificator.processor import TrashClassificator


class TrashSystem:
    def __init__(self, video_source):
        self.cap = cv2.VideoCapture(video_source)
        self.cap.set(3, 1280)
        self.cap.set(4, 720)
        self.trash_classificator_system = TrashClassificator()

    def run(self):
        while self.cap.isOpened():
            success, frame = self.cap.read()
            image = self.trash_classificator_system.frame_processing(frame)
            if not success:
                break

            cv2.imshow("Frame", image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    trash_system = TrashSystem(0)
    trash_system.run()
