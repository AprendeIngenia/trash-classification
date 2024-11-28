from abc import ABC, abstractmethod
from typing import Dict, List
import numpy as np
import cv2


from ultralytics.engine.results import Results
from ultralytics.utils.checks import check_requirements
from ultralytics.utils.plotting import Annotator, colors
from collections import defaultdict

check_requirements("shapely>=2.0.0")


class MaskDrawerInterface(ABC):
    @abstractmethod
    def draw(self, image: np.ndarray, masks: List[np.ndarray], classes: List[int]) -> np.ndarray:
        """draw mask in image"""
        raise NotImplementedError


class MaskDrawer(MaskDrawerInterface):
    def __init__(self):
        self.color_map: Dict[int, tuple] = {0: (255, 0, 0), 1: (255, 255, 0), 2: (200, 200, 200)}

    def draw(self, image: np.ndarray, masks: List[np.ndarray], classes: List[int]) -> np.ndarray:
        overlay = image.copy()
        for mask, cls in zip(masks, classes):
            color = self.color_map.get(cls, (255, 255, 255))
            mask_polygon = np.int32([mask])
            cv2.fillPoly(overlay, mask_polygon, color)
            cv2.polylines(image, mask_polygon, isClosed=True, color=color, thickness=2)
        alpha = 0.5
        return cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)


class BoundingBoxDrawerInterface(ABC):
    def draw(self, image: np.ndarray, boxes: np.ndarray, trash_classes: Dict[int, str], classes: List[int]) -> np.ndarray:
        """ draw bounding box in image"""
        raise NotImplementedError


class BoundingBoxDrawer(BoundingBoxDrawerInterface):
    def __init__(self):
        self.thickness: int = 2

    def draw(self, image: np.ndarray, boxes: np.ndarray, trash_classes: Dict[int, str], classes: List[int]) -> np.ndarray:
        annotator = Annotator(image, self.thickness, trash_classes)
        for box, cls in zip(boxes, classes):
            annotator.box_label(box, trash_classes[cls], color=colors(cls, True))
        return annotator.result()


class TrackDrawerInterface(ABC):
    def draw(self, image: np.ndarray, tracks_ids: List[int], boxes: np.ndarray) -> np.ndarray:
        """draw and save track in image"""
        raise NotImplementedError


class TrackDrawer(TrackDrawerInterface):
    def __init__(self):
        self.track_history = defaultdict(list)
        self.thickness: int = 2

    def draw(self, image: np.ndarray, tracks_ids: List[int], boxes: np.ndarray) -> np.ndarray:
        for track_id, box in zip(tracks_ids, boxes):
            track_line = self.track_history[track_id]
            centroid = (float((box[0] + box[2]) / 2), float((box[1] + box[3]) / 2))
            track_line.append(centroid)

            if len(track_line) > 50:
                track_line.pop(0)

            for i in range(1, len(track_line)):
                cv2.line(image, tuple(map(int, track_line[i - 1])), tuple(map(int, track_line[i])),
                         colors(track_id, True), self.thickness)
        return image


class DrawingInterface(ABC):
    @abstractmethod
    def draw(self, image: np.ndarray, trash_track: Results, trash_classes: Dict[int, str], device):
        raise NotImplementedError


class Drawing(DrawingInterface):
    def __init__(self):
        self.mask_drawer: MaskDrawerInterface = MaskDrawer()
        self.bbox_drawer: BoundingBoxDrawerInterface = BoundingBoxDrawer()
        self.track_drawer: TrackDrawerInterface = TrackDrawer()

    def draw(self, image: np.ndarray, trash_track: Results, trash_classes: Dict[int, str], device):

        masks = trash_track.masks.xy
        boxes = trash_track.boxes.xyxy.cpu()
        tracks_ids = trash_track.boxes.id.int().cpu().tolist()
        clss = trash_track.boxes.cls.cpu().tolist()

        image = self.mask_drawer.draw(image, masks, clss)
        image = self.bbox_drawer.draw(image, boxes, trash_classes, clss)
        image = self.track_drawer.draw(image, tracks_ids, boxes)
        return image
