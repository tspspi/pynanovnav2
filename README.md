# Unofficial NanoVNA v2 USB Python library and tools

__This library is work in progress__

This is a small library that allows to access the NanoVNA v2 vector network
analyzer using Python. It implements the ```VectorNetworkAnalyzer```
base class of [pylabdevs](https://github.com/tspspi/pylabdevs) (since this
is work in progress there is currently an unfinished copy in this repository).

## Tools

### ```nanovnav2fetch```

The ```nanovnav2fetch``` is a very simple command line utility that
allows one to fetch the S00 and S01 parameters as well as phases
from the NanoVNA v2. Note that this utility always depends on ```numpy```
and requires ```matplotlib``` to plot or show the traces.

```
$ nanovnav2fetch --help
usage: nanovnav2fetch [-h] [--port PORT] [--debug] [--s00] [--s01] [--phases]
                      [--show] [--plot [PLOT ...]] [--plottitle PLOTTITLE]
                      [--label00 LABEL00] [--label01 LABEL01] [--npz NPZ]
                      [--start START] [--end END] [--step STEP]

NanoVNA v2 USB fetching utility

optional arguments:
  -h, --help            show this help message and exit
  --port PORT           Port to access the NanoVNA v2 (default: /dev/ttyU0)
  --debug               Enable debug mode on the NanoVNA v2
  --s00                 Gather S00 values
  --s01                 Gather S01 values
  --phases              Gather phases for all enabled channels
  --show                Display the rendered graphics (requires matplotlib)
  --plot [PLOT ...]     Supply filename that will be used to plot graphics
                        (required matplotlib)
  --plottitle PLOTTITLE
                        Title for the plot
  --label00 LABEL00     Label for the S00 parameter
  --label01 LABEL01     Label for the S01 parameter
  --npz NPZ             Dump data into supplied NPZ file
  --start START         Start frequency in Hz (default: 50 MHz)
  --end END             End frequency in Hz (default: 4.4 GHz)
  --step STEP           Step size in Hz (default: 1 kHz)
```
