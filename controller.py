from abc import ABC

import numpy as np

class Controller(ABC):
    def get_control(self, x : np.ndarray) -> np.ndarray:
        pass
