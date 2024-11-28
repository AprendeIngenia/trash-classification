import numpy as np
from typing import Tuple, Dict
from abc import ABC, abstractmethod

from torch import device
from ultralytics.engine.results import Results

from trash_classificator.segmentation.device_manager import DeviceManager
from trash_classificator.segmentation.model_loader import ModelLoader
from trash_classificator.segmentation.models.trash_model import trash_classes


class SegmentationModelInterface(ABC):
    @abstractmethod
    def inference(self, image: np.ndarray) -> Tuple[Results, Dict[int, str], device]:
        pass


class SegmentationModel(SegmentationModelInterface):
    def __init__(self):
        self.device = DeviceManager.get_device()
        self.trash_segmentation_model = ModelLoader(self.device).get_model()

    def inference(self, image: np.ndarray) -> tuple[list[Results], Dict[int, str], device]:
        results = self.trash_segmentation_model.track(image, conf=0.55, verbose=False, persist=True, imgsz=640,
                                                      stream=True)
        return results, trash_classes, self.device

