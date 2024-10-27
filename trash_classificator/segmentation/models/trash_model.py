from typing import List
import os

current_path = os.path.dirname(os.path.abspath(__file__))

trash_model_path: str = current_path + "/trash_segmentation_model.pt"
trash_classes: List[str] = ['cardboard and paper', 'metal', 'plastic']
