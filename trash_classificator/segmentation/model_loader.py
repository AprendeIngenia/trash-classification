import torch
from ultralytics import YOLO
from trash_classificator.segmentation.models.trash_model import (trash_model_path, trash_classes)


class ModelLoader:
    def __init__(self, device: torch.device):
        self.model = YOLO(trash_model_path).to(device)

    def get_model(self) -> YOLO:
        return self.model
