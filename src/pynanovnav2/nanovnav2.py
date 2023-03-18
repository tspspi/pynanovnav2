import sys
sys.path.append("../")


import serial
import atexit
import struct

from time import sleep
# from labdevices import vectornetworkanalyzer
# from vectornetworkanalyzer import VectorNetworkAnalyzer
from pynanovnav2 import vectornetworkanalyzer

from labdevices.exceptions import CommunicationError_ProtocolViolation
from labdevices.exceptions import CommunicationError_Timeout
from labdevices.exceptions import CommunicationError_NotConnected

class NanoVNAV2(vectornetworkanalyzer.VectorNetworkAnalyzer):
	def __init__(
		self,
		port,

		debug = False,
		useNumpy = False
	):
		super().__init__()

		if useNumpy:
			import numpy as np

		self._use_numpy = useNumpy

		self._debug = debug
		self._discard_first_point = False

		self._regs = {
			0x00 : { 'mnemonic' : "sweepStartHz"      , 'regbytes' : 8   , 'fifobytes': None, 'desc' : "Sweep start frequency in Hz"                            , "enable" : True  },
			0x10 : { 'mnemonic' : "sweepStepHz"       , 'regbytes' : 8   , 'fifobytes': None, 'desc' : "Sweep step frequency in Hz"                             , "enable" : True  },
			0x20 : { 'mnemonic' : "sweepPoints"       , 'regbytes' : 2   , 'fifobytes': None, 'desc' : "Number of sweep frequency points"                       , "enable" : True  },
			0x22 : { 'mnemonic' : "valuesPerFrequency", 'regbytes' : 2   , 'fifobytes': None, 'desc' : "Number of values to sample and output per data point"   , "enable" : True  },
			0x26 : { 'mnemonic' : "sampleMode"        , 'regbytes' : 1   , 'fifobytes': None, 'desc' : "0 is VNA data, 1 is raw data and 2 exits USB data mode" , "enable" : False },
			0x30 : { 'mnemonic' : "valuesFIFO"        , 'regbytes' : None, 'fifobytes': 32  , 'desc' : "VNA sweep data points. Writing anything clears the FIFO", "enable" : True  },
			0x40 : { 'mnemonic' : "averageSetting"    , 'regbytes' : 1   , 'fifobytes': None, 'desc' : "Number of samples to average"                           , "enable" : True  },
			0x41 : { 'mnemonic' : "si5351power"       , 'regbytes' : 1   , 'fifobytes': None, 'desc' : "SI5351 power"                                           , "enable" : False },
			0x42 : { 'mnemonic' : "adf4350power"      , 'regbytes' : 1   , 'fifobytes': None, 'desc' : "ADF4350 power"                                          , "enable" : True  },
			0xEE : { 'mnemonic' : "lcddump"           , 'regbytes' : 1   , 'fifobytes': None, 'desc' : "Dump LCD data"                                          , "enable" : False },
			0xF0 : { 'mnemonic' : "deviceVariant"     , 'regbytes' : 1   , 'fifobytes': None, 'desc' : "The device type (0x02 for the NanoVNA v2)"              , "enable" : True  },
			0xF1 : { 'mnemonic' : "protocolVersion"   , 'regbytes' : 1   , 'fifobytes': None, 'desc' : "The protocol version (0x01)"                            , "enable" : True  },
			0xF2 : { 'mnemonic' : "hardwareRevision"  , 'regbytes' : 1   , 'fifobytes': None, 'desc' : "Hardware revision"                                      , "enable" : True  },
			0xF3 : { 'mnemonic' : "firmwareMajor"     , 'regbytes' : 1   , 'fifobytes': None, 'desc' : "Major firmware version"                                 , "enable" : True  },
			0xF4 : { 'mnemonic' : "firmwareMinor"     , 'regbytes' : 1   , 'fifobytes': None, 'desc' : "Minor firmware version"                                 , "enable" : True  }
		}

		self._sweepStartHz = None
		self._sweepStepHz = None
		self._sweepPoints = None
		self._valuesPerFrequency = None
		self._deviceVariant = None
		self._protocolVersion = None
		self._hardwareRevision = None
		self._firmwareVersion = ( None, None )
		self._frequencies = None

		if isinstance(port, serial.Serial):
			self._port = port
			self._portName = None
			self._initialRequests()
		else:
			self._portName = port
			self._port = None

		atexit.register(self.__close)

	# Context management
	def __enter__(self):
		if self._usedConnect:
			raise ValueError("Cannot use context management (with) on a connected port")

		if (self._port is None) and (not (self._portName is None)):
			self._port = serial.Serial(
				self._portName,
				baudrate=9600,
				bytesize=serial.EIGHTBITS,
				parity=serial.PARITY_NONE,
				stopbits=serial.STOPBITS_ONE,
				timeout=15
			)
			self._initialRequests()
		self._usesContext = True
		return self

	def __exit__(self, exc_type, exc_val, exc_tb):
		self.__close()
		self._usesContext = False

	def __close(self):
		atexit.unregister(self.__close)
		if (not (self._port is None)) and (not (self._portName is None)):
			# Leave USB mode
			try:
				self._reg_write(0x26, 2)
			except:
				# Ignore any error while leaving USB mode
				pass

			self._port.close()
			self._port = None

	# Connect and disconnect

	def _connect(self):
		if (self._port is None) and (not (self._portName is None)):
			self._port = serial.Serial(
				self._portName,
				baudrate=9600,
				bytesize=serial.EIGHTBITS,
				parity=serial.PARITY_NONE,
				stopbits=serial.STOPBITS_ONE,
				timeout=15
			)
			self._initialRequests()
		return True

	def _disconnect(self):
		if not (self._port is None):
			self.__close()
		return True

	# Register access

	def _reg_read(self, address):
		if address not in self._regs:
			raise ValueError(f"Address {address} not supported in NanoVNA library")
		if not self._regs[address]['enable']:
			raise ValueError(f"Access to {self._reg[address]['mnemonic']} not enabled in NanoVNA library")

		# Do read ...
		if self._regs[address]['regbytes'] is None:
			raise ValueError(f"Access to {self._reg[address]['mnemonic']} not possible as register")
		elif self._regs[address]['regbytes'] == 1:
			self._port.write(struct.pack('<BB', 0x10, address ))
			value = struct.unpack('<B', self._port.read(1))[0]
		elif self._regs[address]['regbytes'] == 2:
			self._port.write(struct.pack('<BB', 0x11, address ))
			value = struct.unpack('<H', self._port.read(2))[0]
		elif self._regs[address]['regbytes'] == 4:
			self._port.write(struct.pack('<BB', 0x12, address ))
			value = struct.unpack('<I', self._port.read(4))[0]
		elif self._regs[address]['regbytes'] == 8:
			self._port.write(struct.pack('<BB', 0x12, address ))
			valueLow = struct.unpack('<I', self._port.read(4))[0]
			self._port.write(struct.pack('<BB', 0x12, address+4 ))
			valueHigh = struct.unpack('<I', self._port.read(4))[0]
			value = valueHigh * 4294967296 + valueLow
		else:
			raise ValueError(f"Access width {self._regs[address]['regbytes']} not supported for {self._regs[address]['mnemonic']}")

		return value

	def _reg_write(self, address, value):
		if address not in self._regs:
			raise ValueError(f"Address {address} not supported in NanoVNA library")
		if not self._regs[address]['enable']:
			raise ValueError(f"Access to {self._reg[address]['mnemonic']} not enabled in NanoVNA library")

		# Do write ...

		if address not in self._regs:
			raise ValueError(f"Address {address} not supported in NanoVNA library")
		if not self._regs[address]['enable']:
			raise ValueError(f"Access to {self._reg[address]['mnemonic']} not enabled in NanoVNA library")

		if self._regs[address]['regbytes'] is None:
			raise ValueError(f"Access to {self._reg[address]['mnemonic']} not possible as register")
		elif self._regs[address]['regbytes'] == 1:
			self._port.write(struct.pack('<BBB', 0x20, address, value ))
		elif self._regs[address]['regbytes'] == 2:
			self._port.write(struct.pack('<BBH', 0x21, address, value ))
		elif self._regs[address]['regbytes'] == 4:
			self._port.write(struct.pack('<BBI', 0x22, address, value ))
		elif self._regs[address]['regbytes'] == 8:
			self._port.write(struct.pack('<BBQ', 0x23, address, value ))
		else:
			raise ValueError(f"Access width {self._regs[address]['regbytes']} not supported for {self._regs[address]['mnemonic']}")

	def _op_indicate(self):
		if self._port is None:
			raise CommunicationError_NotConnected("Device is not connected")

		self._port.write(struct.pack('<B', 0x0D ))
		resp = struct.unpack('<B', self._port.read(1))[0]

		return resp

	def _op_nop(self):
		if self._port is None:
			raise CommunicationError_NotConnected("Device is not connected")

		self._port.write(struct.pack('<B', 0x00))

	def _initialRequests(self):
		# Send a few no-operation bytes to terminate any lingering
		# commands ...
		for i in range(64):
			self._op_nop()

		# Now read in a loop until we are not able to read any more data
		while True:
			dta = self._port.read(1)
			if not dta:
				break

		# Check if indicate really returned ASCII 2 ...
		indicateResult = self._op_indicate()
		if indicateResult != 0x32:
			raise CommunicationError_ProtocolViolation(f"Would expect device to report version 2 (0x32, 50), received {indicateResult}")

		# Write initial values
		#   Initial frequency: 500 MHz (applying first point discard)
		#   4 MHz steps
		#   101 samples (100 + discarded first point)
		#   1 value per frequency
		self._reg_write(0x00, int(500e6)-4)
		self._reg_write(0x10, int(35))
		self._reg_write(0x20, 101)
		self._reg_write(0x22, 1)

		if self._use_numpy:
			import numpy as np
			if self._discard_first_point:
				self._frequencies = np.linspace(500e6 - 35, 500e6 + 101 * 35, 101+1)
			else:
				self._frequencies = np.linspace(500e6, 500e6 + 101 * 35, 101)
		else:
			self._frequencies = []
			if self._discard_first_point:
				for i in range(101+1):
					self._frequencies.append(500e6 - 4 + i * 4)
			else:
				for i in range(101):
					self._frequencies.append(500e6 + i * 4)

		# Load current state from registers ...
		self._sweepStartHz = self._reg_read(0x00)
		self._sweepStepHz = self._reg_read(0x10)
		self._sweepPoints = self._reg_read(0x20)
		self._valuesPerFrequency = self._reg_read(0x22)
		self._deviceVariant = self._reg_read(0xF0)
		self._protocolVersion = self._reg_read(0xF1)
		self._hardwareRevision = self._reg_read(0xF2)
		self._firmwareVersion = ( self._reg_read(0xF3), self._reg_read(0xF4) )
		if self._debug:
			print(f"Initial settings:")
			print(f"\tSweep start frequency: {self._sweepStartHz}")
			print(f"\tSweep step frequency:  {self._sweepStepHz}")
			print(f"\tSweep points:          {self._sweepPoints}")
			print(f"\tValues per frequency:  {self._valuesPerFrequency}")
			print(f"\tDevice variant:        {self._deviceVariant}")
			print(f"\tProtocol version:      {self._protocolVersion}")
			print(f"\tHardware revision:     {self._hardwareRevision}")
			print(f"\tFirmware version:      {self._firmwareVersion}")

		if self._firmwareVersion[0] == 0xFF:
			raise CommunicationError_ProtocolViolation("Device is in DFU mode")

	# Overriden methods from base class

	def _get_id(self):
		if self._port is None:
			raise CommunicationError_NotConnected("Device is not connected")

		return {
			"title" : "NanoVNA V2",
			"firmware" : self._firmwareVersion,
			"variant" : self._deviceVariant,
			"protocol" : self._protocolVersion
		}

	def _set_sweep_start_stop_points(self, start, stop, nsamps):
		start = int(start)
		stop = int(stop)
		if nsamps > 1:
			stepsize = int((stop - start) / int(nsamps-1))
		else:
			stepsize = 0
		nsteps = int(nsamps)

		return self._set_sweep_params(start, stepsize, nsteps, self._valuesPerFrequency)

	def _set_sweep_start_size_n(self, start, stepsize, nsamps):
		start = int(start)
		stepsize = int(stepsize)
		nsamps = int(nsamps)

		return self._set_sweep_params(start, stepsize, nsamps, self._valuesPerFrequency)

	def _set_samples_per_point(self, nsamples):
		nsamples = int(nsamples)
		if nsamples <= 0:
			raise ValueError("Number of samples has to be a positive integer")

		return self._set_sweep_params(self._sweepStartHz, self._sweepStepHz, self._sweepPoints, nsamples, applyHack = False)

	def __complex_divide(self, a, b):
		# Perform complex number division
		#
		# (a.re + i * a.im) / (b.re + i * b.im)
		# = (a.re + i * a.im)*(b.re - i*b.im) / ((b.re + i * b.im)*(b.re - i*b.im))
		# = (a.re * b.re - i * a.re * b.im + i*a.im*b.re + a.im*b.im) / (b.re*b.re + b.im * b.im)
		# = (a.re * b.re  + a.im*b.im + i (a.im*b.re + a.re * b.im)) / (b.re*b.re + b.im * b.im)

		return (
			a[0] * b[0] + a[1] * b[1] / (b[0] * b[0] + b[1] * b[1]),
			a[1] * b[0] + a[0] * b[1] / (b[0] * b[0] + b[1] * b[1])
		)

	def _query_trace(self):
		if self._port is None:
			raise CommunicationError_NotConnected("Device is not connected, failed to query data")

		# Clear FIFO by a simple write ...
		self._port.write(struct.pack('<BBB', 0x20, 0x30, 0x00 ))

		# Now read data
		nPointsToRead = self._valuesPerFrequency * self._sweepPoints
		nDataPoints = nPointsToRead

		# Read in batches of <= 255 samples ...
		while nPointsToRead > 0:
			batchPoints = min(nPointsToRead, 255)

			self._port.write(struct.pack('<BBB', 0x18, 0x30, batchPoints))
			nBytesToRead = 32 * batchPoints
			nBytesRead = 0
			alldata = None
			while (alldata is None) or (nBytesRead < nBytesToRead):
				datanew = self._port.read(nBytesToRead - nBytesRead)
				if datanew is None:
					raise CommunicationError_Timeout("No data received from FIFO")

				if alldata is not None:
					alldata = alldata + datanew
				else:
					alldata = datanew

				nBytesRead = nBytesRead + len(datanew)

			nPointsToRead = nPointsToRead - batchPoints

		# Decode all datapoints

		if self._use_numpy:
			import numpy as np

			pkgdata = {
				"freq" : np.full((nDataPoints), np.nan),

				"fwd0" : np.full((nDataPoints), np.nan, dtype = complex),
				"rev0" : np.full((nDataPoints), np.nan, dtype = complex),
				"rev1" : np.full((nDataPoints), np.nan, dtype = complex),

				"s00raw" : np.full((nDataPoints), np.nan, dtype = complex),
				"s01raw" : np.full((nDataPoints), np.nan, dtype = complex)
			}
		else:
			pkgdata = {
				"freq" : [],
				"fwd0" : [],
				"rev0" : [],
				"rev1" : [],

				"s00raw" : [],
				"s01raw" : []
			}
			for _ in range(nDataPoints):
				pkgdata["freq"].append(0.0)

				pkgdata["fwd0"].append(0.0)
				pkgdata["rev0"].append(0.0)
				pkgdata["rev1"].append(0.0)

				pkgdata["s00raw"].append(0.0)
				pkgdata["s01raw"].append(0.0)

		if self._discard_first_point:
			nDataPoints = nDataPoints - 1
			alldata = alldata[32 : ]

		for iPoint in range(nDataPoints):
			packet = alldata[iPoint * 32 : (iPoint+1) * 32]

			# value = struct.unpack('<H', self._port.read(2))[0]
			fwd0Re, fwd0Im, rev0Re, rev0Im, rev1Re, rev1Im, freqIndex, _, _ = struct.unpack('<iiiiiiHHI', packet)

			pkgdata["freq"][freqIndex] = self._frequencies[freqIndex]

			if not self._use_numpy:
				pkgdata["fwd0"][freqIndex] = (fwd0Re, fwd0Im)
				pkgdata["rev0"][freqIndex] = (rev0Re, rev0Im)
				pkgdata["rev1"][freqIndex] = (rev1Re, rev1Im)
			else:
				pkgdata["fwd0"][freqIndex] = fwd0Re + 1j*fwd0Im
				pkgdata["rev0"][freqIndex] = rev0Re + 1j*rev0Im
				pkgdata["rev1"][freqIndex] = rev1Re + 1j*rev1Im

			# Recover (raw, uncalibrated) S00 and S01 by performing phase and amplitude
			# correction of reflected and transmitted signal (normalize by output signal)

			if not self._use_numpy:
				pkgdata["s00raw"][freqIndex] = self.__complex_divide( (rev0Re, rev0Im), (fwd0Re, fwd0Im) )
				pkgdata["s01raw"][freqIndex] = self.__complex_divide( (rev1Re, rev1Im), (fwd0Re, fwd0Im) )
			else:
				pkgdata["s00raw"] = pkgdata["rev0"] / pkgdata["fwd0"]
				pkgdata["s01raw"] = pkgdata["rev1"] / pkgdata["fwd0"]

		return pkgdata
	# Internal methods


	def _set_sweep_params(
		self,
		start = None,
		stepsize = None,
		nsteps = None,
		npoints = None,

		applyHack = True
	):
		if start is not None:
			start = int(start)
		else:
			start = self._sweepStartHz

		if stepsize is not None:
			stepsize = int(stepsize)
		else:
			stepsize = self._sweepStepHz

		if nsteps is not None:
			nsteps = int(nsteps)
		else:
			nsteps = self._sweepPoints

		if npoints is not None:
			npoints = int(npoints)
		else:
			npoints = self._valuesPerFrequency

		if self._discard_first_point and applyHack:
			# We discard the first point
			start = start - stepsize
			nsteps = nsteps + 1

		if (start <= 0) or (stepsize <= 0) or (nsteps <= 0) or (npoints <= 0):
			raise ValueError("NanoVNA v2 does not accept sweep parameters smaller or equal to zero")

		end = start + (nsteps - 1) * stepsize

		if start < 50e3:
			raise ValueError("NanoVNA v2 does not support sweep range starting lower than 50 kHz")
		if start > 4e9:
			raise ValueError("NanoVNA v2 does not support sweep range starting higher than 4 GHz")
		if end < 50e3:
			raise ValueError("NanoVNA v2 does not support sweep range ending lower than 50 kHz")
		if end > 4e9:
			raise ValueError("NanoVNA v2 does not support sweep range ending higher than 4 GHz")

		self._reg_write(0x00, start)
		self._reg_write(0x10, stepsize)
		self._reg_write(0x20, nsteps)
		self._reg_write(0x22, npoints)

		# Read back values and update state

		read_sweepStartHz = self._reg_read(0x00)
		read_sweepStepHz = self._reg_read(0x10)
		read_sweepPoints = self._reg_read(0x20)
		read_valuesPerFrequency = self._reg_read(0x22)

		if not ((read_sweepStartHz == start) and (read_sweepStepHz == stepsize) and (read_sweepPoints == nsteps) and (read_valuesPerFrequency == npoints)):
			# Setting failed ...
			return False

		if self._use_numpy:
			import numpy as np
			self._frequencies = np.linspace(read_sweepStartHz, read_sweepStartHz + read_sweepPoints * read_sweepStepHz, read_sweepPoints)
		else:
			self._frequencies = []
			for i in range(read_sweepPoints):
				self._frequencies.append(read_sweepStartHz + i * read_sweepStepHz)

		self._valuesPerFrequency = read_valuesPerFrequency
		self._sweepStepHz = read_sweepStepHz
		self._sweepPoints = read_sweepPoints
		self._sweepStartHz = read_sweepStartHz
		if self._discard_first_point and applyHack:
			self._sweepStartHz = self._sweepStartHz + self._sweepStepHz

		return True

if __name__ == "__main__":
	with NanoVNAV2("/dev/ttyU0", debug = True, useNumpy = True) as vna:
		print(f"Indicate returns {vna._op_indicate()}")
		print(f"ID returned {vna._get_id()}")
		#vna._set_sweep_start_size_n(500e6, 1e6, 101)
		vna._set_sweep_start_size_n(500e6, 1e3, 101)
		trace = vna._query_trace()

		import numpy as np
		import matplotlib.pyplot as plt
		fig, ax = plt.subplots(2, 2, figsize=(6.4*2, 4.8*2))
		ax[0][0].set_title("Magnitude")
		ax[0][0].plot(trace["freq"], np.absolute(trace["s01raw"]), label = "S01")
		ax[0][0].plot(trace["freq"], np.absolute(trace["s00raw"]), label = "S00")
		ax[0][0].grid()
		ax[0][0].legend()
		ax[0][1].set_title("Phase")
		ax[0][1].plot(trace["freq"], np.angle(trace["s01raw"]), label = "S01")
		ax[0][1].plot(trace["freq"], np.angle(trace["s00raw"]), label = "S00")
		ax[0][1].grid()
		ax[0][1].legend()

		ax[1][0].set_title("Transmitted (fwd)")
		ax[1][0].plot(trace["freq"], np.real(trace["fwd0"]), label = "I")
		ax[1][0].plot(trace["freq"], np.imag(trace["fwd0"]), label = "Q")
		ax[1][0].grid()
		ax[1][0].legend()

		ax[1][1].set_title("S00, S01 (raw)")
		ax[1][1].plot(trace["freq"], np.real(trace["rev1"]), label = "I S01")
		ax[1][1].plot(trace["freq"], np.imag(trace["rev1"]), label = "Q S01")
		ax[1][1].plot(trace["freq"], np.real(trace["rev0"]), label = "I S00")
		ax[1][1].plot(trace["freq"], np.imag(trace["rev0"]), label = "Q S00")
		ax[1][1].grid()
		ax[1][1].legend()
		plt.show()
		pass