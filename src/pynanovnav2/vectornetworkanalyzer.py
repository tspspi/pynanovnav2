import atexit

from enum import Enum

class VectorNetworkAnalyzer_AverageMode(Enum):
    POINT_BY_POINT = 1
    SWEEP_BY_SWEEP = 2

class VectorNetworkAnalyzer:
    def __init__(
        self,

        frequencyRange = ( None, None ),

        frequencyStepRange = ( None, None ),
        frequencyStepDiscrete = None,

        attenuatorRange = ( None, None ),
        attenuatorDiscrete = None,
        preampRange = ( None, None ),
        preampDiscrete = None,

        trackingGeneratorAmplitude = ( None, None )
    ):
        self._usesContext = False
        self._usedConnect = False

    # Overridden methods:

    def _get_id(self):
        raise NotImplementedException("Method not implemented by device")
    def _set_sweep_range(self, start, stop, step = None):
        raise NotImplementedException("Method not implemented by device")
    def _query_trace(self):
        raise NotImplementedException("Method not implemented by device")
    def _set_average(self, naverages, averageMode = VectorNetworkAnalyzer_AverageMode.POINT_BY_POINT):
        raise NotImplementedException("Method not implemented by device")

    # Exposed methods ...
