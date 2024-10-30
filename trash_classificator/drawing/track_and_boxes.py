from abc import ABC, abstractmethod
from typing import Tuple
import numpy as np
import cv2

from ultralytics.engine.results import Results
from ultralytics.utils.checks import check_requirements
from ultralytics.utils.plotting import Annotator, colors
from shapely.geometry import LineString, Point, Polygon
from collections import defaultdict

check_requirements("shapely>=2.0.0")


class DrawDetectionsInterface(ABC):
    @abstractmethod
    def draw(self, image: np.ndarray, trash_track: Results, trash_classes: list, device):
        raise NotImplementedError


class DrawDetections(DrawDetectionsInterface):
    def __init__(self):
        # image and annotation information
        self.im0 = None
        self.thickness: int = 2
        self.view_img: bool = True
        self.view_in_counts: bool = True
        self.view_out_counts: bool = True

        # object information
        self.in_counts: int = 0
        self.out_counts: int = 0
        self.counts_ids: list = []
        self.class_wise_count: dict = {}

        # track info
        self.track_history = defaultdict(list)
        self.draw_tracks: bool = True

    def draw(self, image: np.ndarray, trash_track: Results, trash_classes: list, device):
        boxes = trash_track.boxes.xyxy.cpu()
        tracks_ids = trash_track.boxes.id.int().cpu().tolist()
        clss = trash_track.boxes.cls.cpu().tolist()

        annotator = Annotator(image, self.thickness, trash_classes)

        for box, track_id, cls in zip(boxes, tracks_ids, clss):
            # draw bbox
            annotator.box_label(box, str(trash_classes[cls]), color=colors(cls, True))

            # object history
            track_line = self.track_history[track_id]
            track_line.append((float((box[0] + box[2]) / 2), float((box[1] + box[3]) / 2)))

            if len(track_line) > 50:
                track_line.pop(0)

            # draw tracks
            if self.draw_tracks:
                annotator.draw_centroid_and_tracks(
                    track_line,
                    color=colors(int(track_id), True),
                    track_thickness=self.thickness,
                )
        return image
