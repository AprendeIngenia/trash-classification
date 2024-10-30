import numpy as np
from typing import Tuple, List, Dict, Any

from trash_classificator.segmentation.main import (SegmentationModelInterface, SegmentationModel)
from trash_classificator.drawing.track_and_boxes import (DrawDetectionsInterface, DrawDetections)


class TrashClassificator:
    def __init__(self):
        self.segmentation: SegmentationModelInterface = SegmentationModel()
        self.draw_detections: DrawDetectionsInterface = DrawDetections()

    def frame_processing(self, image: np.ndarray):

        # step 1: trash segmentation
        trash_image = image.copy()
        trash_track, trash_classes, device = self.segmentation.inference(trash_image)

        for trash in trash_track:
            if trash.boxes.id is None:
                return image, 'No trash detected'

            # step 2: draw detections
            image_draw = image.copy()
            image_draw = self.draw_detections.draw(image_draw, trash, trash_classes, device)

            return image_draw, 'Trash detected'

        return image, 'Trash detected'
