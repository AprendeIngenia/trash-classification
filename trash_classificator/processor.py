import numpy as np
from typing import Tuple, List, Dict, Any

from trash_classificator.segmentation.main import (SegmentationModelInterface, SegmentationModel)


class TrashClassificator:
    def __init__(self):
        self.segmentation: SegmentationModelInterface = SegmentationModel()

    def frame_processing(self, image: np.ndarray):
        return image
