import atexit

class VectorNetworkAnalyzer:
	def __init__(
		self
	):
		self._usesContext = False
		self._usedConnect = False

	# Overridden methods:

	def _get_id(self):
		raise NotImplementedException("Method not implemented by device")
	def _set_sweep_start_stop_n(self, start, stop, nsamps):
		raise NotImplementedException("Method not implemented by device")
	def _set_sweep_start_size_n(self, start, stepsize, nsamps):
		raise NotImplementedException("Method not implemented by device")
	def _set_samples_per_point(self, nsamples):
		raise NotImplementedException("Method not implemented by device")
	def _query_trace(self):
		raise NotImplementedException("Method not implemented by device")