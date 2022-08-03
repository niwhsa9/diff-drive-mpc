from abc import ABC, abstractmethod

import numpy as np

class Controller(ABC):
    @abstractmethod
    def get_control(self, x : np.ndarray) -> np.ndarray:
        pass

class DummyController(Controller):
    def get_control(self, _) -> np.ndarray:
        return np.array([0, 0])
