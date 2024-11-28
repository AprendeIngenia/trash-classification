from typing import Dict
import os

current_path = os.path.dirname(os.path.abspath(__file__))

trash_model_path: str = current_path + "/trash_segmentation_model_v2.pt"
trash_classes: Dict[int, str] = {0: 'cardboard and paper', 1: 'metal', 2: 'plastic'}
