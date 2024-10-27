import torch
import logging as log
log.basicConfig(level=log.INFO, format="%(asctime)s - %(levelname)s - %(message)s")


class DeviceManager:
    @staticmethod
    def get_device() -> torch.device:
        if torch.backends.mps.is_available():
            device = torch.device("mps")
        elif torch.cuda.is_available():
            device = torch.device("cuda")
        else:
            device = torch.device("cpu")

        DeviceManager.log_device(device)
        return device

    @staticmethod
    def log_device(device: torch.device):
        if device.type == "mps":
            device_name = "MPS"
        elif device.type == "cuda":
            device_name = torch.cuda.get_device_name(device)
        else:
            device_name = "CPU"
        log.info(f"Model is using device: {device_name}")
