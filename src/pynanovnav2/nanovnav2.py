import sys
sys.path.append("../")

import serial
import atexit
import struct
import math

import logging

from time import sleep
# from labdevices import vectornetworkanalyzer
# from vectornetworkanalyzer import VectorNetworkAnalyzer
from pynanovnav2 import vectornetworkanalyzer

from labdevices.exceptions import CommunicationError_ProtocolViolation
from labdevices.exceptions import CommunicationError_Timeout
from labdevices.exceptions import CommunicationError_NotConnected

# Spectrum analyzer wrapper class
#
# This uses the NanoVNA V2 only on port2 (since the tracking generator
# is not disable-able). It's assumed that port1 is termianted with 50 Ohms

class NanoVNAV2SpectrumAnalyzerPort2:
    def __init__(
        self,
        port,

        logger = None,
        debug = False,
        useNumpy = False,
        loglevel = logging.ERROR
    ):
        self._vna = NanoVNAV2(port, logger, debug, useNumpy, loglevel)


class NanoVNAV2(vectornetworkanalyzer.VectorNetworkAnalyzer):
    def __init__(
        self,
        port,

        logger = None,
        debug = False,
        useNumpy = False,
        loglevel = logging.ERROR
    ):
        super().__init__(
            frequencyRange = ( 50e3, 4400e6 ),
            frequencyStepRange = ( 1e3, 10e6 ),
            attenuatorRange = ( 0, 0 ),
            preampRange = ( 0, 0 ),
            trackingGeneratorAmplitude = ( -7, -7 )
        )

        if useNumpy:
            import numpy as np

        if logger is not None:
            self._logger = logger
        else:
            self._logger = logging.getLogger(__name__)
            self._logger.addHandler(logging.StreamHandler(sys.stderr))
            self._logger.setLevel(loglevel)

        self._use_numpy = useNumpy

        self._debug = debug
        self._discard_first_point = True

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
        if False:
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

    def _set_sweep_range(self, start, stop, step = 50e3):
        # Calculate number of segments, round up to next integer
        # and calculate new end of sweep

        wndPoints = 101
        frqStart = start
        if self._discard_first_point:
            # We have to modify start one point lower and have to account for
            # overlapping windows ... so only 100 points instead of 101 per window since
            # we include the previous last one
            start = start - step
            wndPoints = 100

        if int(step) != step:
            raise ValueError("Step size has to be an integer value")
        if int(start) != start:
            raise ValueError("Start has to be an integer value")
        if int(stop) != stop:
            raise ValueError("Stop has to be an integer value")

        if float(start) < 50e3:
            raise ValueError("Supported frequency range is above 50 kHz")
        if float(stop) <= float(start):
            raise ValueError("Stop frequency has to be above start frequency")
        if float(start) > 3e9:
            raise ValueError("Stop frequency has to be below 3 GHz")
        if int(step) < 1e3:
            raise ValueError("Step size has to be 1 kHz or larger")

        nPointsTotal = int(int((stop - start) / int(step)))
        nSegments = int(nPointsTotal / wndPoints)
        if nSegments == 0:
            nSegments = 1
        else:
            nSegments = math.ceil(nSegments)

        # Might differ ...
        nPointsTotal = nSegments * wndPoints

        stop = start + step * wndPoints * nSegments

        self._sweepStartHz = start
        self._sweepStepHz = step
        self._sweepPoints = wndPoints
        self._sweepSegments = nSegments
        self._valuesPerFrequency = 1

        # Create frequencies array ...
        if self._use_numpy:
            import numpy as np
            self._frequencies = np.linspace(frqStart, frqStart + wndPoints*nSegments*step, (wndPoints*nSegments)+1)
        else:
            self._frequencies = []
            for i in range(wndPoints * nSegments):
                self._frequencies.append(start + i * step)

        return True

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
            raise CommunicationError_NotConnected("Device it not connected, failed to query data")

        currentStart = self._sweepStartHz
        currentValuesPerFreq = self._valuesPerFrequency
        freqBaseIndex = 0

        if self._use_numpy:
            import numpy as np
            pkgdata = {
                "freq" : np.asarray([]), # np.full((self._sweepPoints * self._sweepSegments), np.nan),

                "fwd0" : np.asarray([]), # np.full((self._sweepPoints * self._sweepSegments), np.nan, dtype = complex),
                "rev0" : np.asarray([]), # np.full((self._sweepPoints * self._sweepSegments), np.nan, dtype = complex),
                "rev1" : np.asarray([]), # np.full((self._sweepPoints * self._sweepSegments), np.nan, dtype = complex),

                "s00raw" : np.asarray([]), # np.full((self._sweepPoints * self._sweepSegments), np.nan, dtype = complex),
                "s01raw" : np.asarray([])  # np.full((self._sweepPoints * self._sweepSegments), np.nan, dtype = complex)
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
            for _ in range(self._sweepPoints * self._sweepSegments):
                pkgdata["freq"].append(0.0)
                pkgdata["fwd0"].append(0.0)
                pkgdata["rev0"].append(0.0)
                pkgdata["rev1"].append(0.0)
                pkgdata["s00raw"].append(0.0)
                pkgdata["s01raw"].append(0.0)

        # Now iterate over each segment ...

        for iSegment in range(self._sweepSegments):
            if False:
                print(f"Sweep segment:")
                print(f"\tStart: {currentStart}")
                print(f"\tStep:  {self._sweepStepHz}")
                print(f"\tPoints:101")
                print(f"\tAvg:   {self._valuesPerFrequency}")
            # Set parameters for sweep
            realSweepPoints = self._sweepPoints
            if self._discard_first_point:
                realSweepPoints = realSweepPoints + 1

            self._reg_write(0x00, int(currentStart))
            self._reg_write(0x10, int(self._sweepStepHz))
            self._reg_write(0x20, realSweepPoints)
            self._reg_write(0x22, int(self._valuesPerFrequency))

            # Clear FIFO ...
            self._port.write(struct.pack('<BBB', 0x20, 0x30, 0x00))

            # Read data ...
            nPointsToRead = self._valuesPerFrequency * realSweepPoints
            nDataPoints = nPointsToRead
            while nPointsToRead > 0:
                batchPoints = min(nPointsToRead, 255)
                self._port.write(struct.pack('<BBB', 0x18, 0x30, batchPoints))
                nBytesToRead = 32 * batchPoints
                nBytesRead = 0
                alldata = None
                while (alldata is None) or (nBytesRead < nBytesToRead):
                    datanew = self._port.read(nBytesToRead - nBytesRead)
                    if datanew is None:
                        raise CommunicationError_Timeout("Failed to receive FIFO data")
                    if alldata is not None:
                        alldata = alldata + datanew
                    else:
                        alldata = datanew

                    nBytesRead = nBytesRead + len(datanew)

                nPointsToRead = nPointsToRead - batchPoints

            # If we have to discard the first data point - drop ...
            if self._discard_first_point:
                nDataPoints = nDataPoints - 1
                alldata = alldata[32 : ]

            # Decode data points of _this_ packet
            if self._use_numpy:
                import numpy as np
                newpkgdata = {
                    "freq" : np.full((nDataPoints), np.nan),

                    "fwd0" : np.full((nDataPoints), np.nan, dtype = complex),
                    "rev0" : np.full((nDataPoints), np.nan, dtype = complex),
                    "rev1" : np.full((nDataPoints), np.nan, dtype = complex),

                    "s00raw" : np.full((nDataPoints), np.nan, dtype = complex),
                    "s01raw" : np.full((nDataPoints), np.nan, dtype = complex)
                }
            else:
                newpkgdata = {
                    "freq" : [],
                    "fwd0" : [],
                    "rev0" : [],
                    "rev1" : [],
                    "s00raw" : [],
                    "s01raw" : []
                }
                for _ in range(nDataPoints):
                    newpkgdata["freq"].append(0.0)
                    newpkgdata["fwd0"].append(0.0)
                    newpkgdata["rev0"].append(0.0)
                    newpkgdata["rev1"].append(0.0)
                    newpkgdata["s00raw"].append(0.0)
                    newpkgdata["s01raw"].append(0.0)

            for iPoint in range(nDataPoints):
                packet = alldata[iPoint * 32 : (iPoint + 1) * 32]
                fwd0Re, fwd0Im, rev0Re, rev0Im, rev1Re, rev1Im, freqIndex, _, _ = struct.unpack('<iiiiiiHHI', packet)

                if self._discard_first_point:
                    freqIndex = freqIndex - 1

                newpkgdata["freq"][freqIndex] = self._frequencies[freqIndex + freqBaseIndex]
                if not self._use_numpy:
                    newpkgdata["fwd0"][freqIndex] = (fwd0Re, fwd0Im)
                    newpkgdata["rev0"][freqIndex] = (rev0Re, rev0Im)
                    newpkgdata["rev1"][freqIndex] = (rev1Re, rev1Im)
                else:
                    newpkgdata["fwd0"][freqIndex] = fwd0Re + 1j*fwd0Im
                    newpkgdata["rev0"][freqIndex] = rev0Re + 1j*rev0Im
                    newpkgdata["rev1"][freqIndex] = rev1Re + 1j*rev1Im

                # Recover uncalibrated raw data for S parameters by taking into account
                # transmitted signal amplitude and phase

                if not self._use_numpy:
                    newpkgdata["s00raw"][freqIndex] = self.__complex_divide( (rev0Re, rev0Im), (fwd0Re, fwd0Im) )
                    newpkgdata["s01raw"][freqIndex] = self.__complex_divide( (rev1Re, rev1Im), (fwd0Re, fwd0Im) )
                else:
                    newpkgdata["s00raw"][freqIndex] = newpkgdata["rev0"][freqIndex] / newpkgdata["fwd0"][freqIndex]
                    newpkgdata["s01raw"][freqIndex] = newpkgdata["rev1"][freqIndex] / newpkgdata["fwd0"][freqIndex]

            # Merge to global packet data ...
            if self._use_numpy:
                for fld in [ "freq", "fwd0", "rev0", "rev1", "s00raw", "s01raw" ]:
                    pkgdata[fld] = np.concatenate((pkgdata[fld], newpkgdata[fld]))
            else:
                for fld in [ "freq", "fwd0", "rev0", "rev1", "s00raw", "s01raw" ]:
                    pkgdata[fld] = pkgdata[fld] + newpkgdata[fld]

            # Update for next sweep window

            freqBaseIndex = freqBaseIndex + len(newpkgdata["freq"])
            currentStart = currentStart + self._sweepStepHz * 101
            if self._discard_first_point:
                currentStart = currentStart - self._sweepStepHz

        if self._use_numpy:
            pkgdata["s00rawdbm"] = np.log10(np.absolute(pkgdata["s00raw"])) * 20
            pkgdata["s01rawdbm"] = np.log10(np.absolute(pkgdata["s01raw"])) * 20
            pkgdata["s00rawphase"] = np.angle(pkgdata["s00raw"])
            pkgdata["s01rawphase"] = np.angle(pkgdata["s01raw"])
        else:
            #ToDo ... without numpy ...
            pass

        return pkgdata


if __name__ == "__main__":
    with NanoVNAV2("/dev/ttyU0", debug = True, useNumpy = True) as vna:
        print(f"ID returned {vna._get_id()}")

        import numpy as np
        import matplotlib.pyplot as plt

        vna._set_sweep_range(100e6, 500e6, 50e3)
        data = vna._query_trace()

        fig, ax = plt.subplots(2, figsize=(6.4*2, 4.8*4))
        ax[0].plot(data["freq"]/1e6, data["s00rawdbm"], label = "S00")
        ax[0].plot(data["freq"]/1e6, data["s01rawdbm"], label = "S01")
        ax[0].set_xlabel("Frequency [MHz]")
        ax[0].set_ylabel("Power [dB]")
        #ax[0].set_title("S00 of microcoil on outside flange")
        ax[0].grid()
        ax[0].legend()

        ax[1].plot(data["freq"]/1e6, data["s00rawphase"], label = "Phase S00")
        ax[1].plot(data["freq"]/1e6, data["s01rawphase"], label = "Phase S01")
        ax[1].set_xlabel("Frequency [MHz]")
        ax[1].set_ylabel("Phase [rad]")
        ax[1].grid()
        ax[1].legend()

        np.savez("data.npz", **data)

        plt.show()

if __name__ == "__main__OLD":
    with NanoVNAV2("/dev/ttyU0", debug = True, useNumpy = True) as vna:
        print(f"Indicate returns {vna._op_indicate()}")
        print(f"ID returned {vna._get_id()}")
        #vna._set_sweep_start_size_n(500e6, 1e6, 101)
        #vna._set_sweep_start_size_n(500e6, 1e3, 101)
        #vna._set_sweep_start_size_n(100e6, 1e3, )

        import numpy as np
        import matplotlib.pyplot as plt 

        start = int(100e6)
        frqs = []
        s01, s00 = [], []
        step = 50e3
        
        while(start < 500e6):
            print(f"{start} ... ")
            vna._set_sweep_start_size_n(start, step, 101)
            trace = vna._query_trace()
            start = start + int(101*step)

            frqs.append(trace["freq"])
            s01.append(np.absolute(trace["s01raw"]))
            s00.append(np.absolute(trace["s00raw"]))

        frqs = np.concatenate(frqs)
        s00 = np.concatenate(s00)
        s01 = np.concatenate(s01)

        fig, ax = plt.subplots(figsize=(6.4, 4.8))
        ax.plot(frqs / 1e6, s00, label = "S00")
        ax.plot(frqs / 1e6, s01, label = "S01")
        ax.grid()
        ax.legend()
        ax.set_xlabel("Frequency [MHz]")
        ax.set_ylabel("Power")
        plt.show()

        np.savez("data.npz", frqs = frqs, s00 = s00, s01 = s01)

        if False:
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
