import numpy as np
from typing import Tuple, List
from abc import ABC, abstractmethod
from ultralytics.engine.results import Results

from trash_classificator.segmentation.device_manager import DeviceManager
from trash_classificator.segmentation.model_loader import ModelLoader


class SegmentationModelInterface(ABC):
    @abstractmethod
    def trash_segmentation(self, image: np.ndarray) -> Tuple[Results, list]:
        pass


class SegmentationModel(SegmentationModelInterface):
    def __init__(self):
        self.device = DeviceManager.get_device()
        self.trash_segmentation_model = ModelLoader(self.device).get_model()

    def trash_segmentation(self, image: np.ndarray) -> Tuple[List[Results], List[str]]:
        results = self.trash_segmentation_model.track(image, conf=0.60, verbose=False, persist=True, half=True,
                                                      imgsz=640, stream=True)
        return results, self.trash_segmentation_model.names

