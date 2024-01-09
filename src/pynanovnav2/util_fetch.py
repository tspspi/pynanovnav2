from pynanovnav2.nanovnav2 import NanoVNAV2

import numpy as np

import argparse
import sys

def _parseArguments():
    ap = argparse.ArgumentParser(description = "NanoVNA v2 USB fetching utility")

    ap.add_argument('--port', type=str, required=False, default="/dev/ttyU0", help="Port to access the NanoVNA v2 (default: /dev/ttyU0)")
    ap.add_argument('--debug', action='store_true', help="Enable debug mode on the NanoVNA v2")

    ap.add_argument('--s00', action='store_true', help="Gather S00 values")
    ap.add_argument('--s01', action='store_true', help="Gather S01 values")
    ap.add_argument('--phases', action='store_true', help="Gather phases for all enabled channels")

    ap.add_argument('--show', action='store_true', help="Display the rendered graphics (requires matplotlib)")

    ap.add_argument('--plot', type=str, action='append', nargs='*', required=False, default=None, help="Supply filename that will be used to plot graphics (required matplotlib)")
    ap.add_argument('--plottitle', type=str, required=False, default="NanoVNA v2", help="Title for the plot")
    ap.add_argument('--label00', type=str, required=False, default="S00", help="Label for the S00 parameter")
    ap.add_argument('--label01', type=str, required=False, default="S01", help="Label for the S01 parameter")

    ap.add_argument('--npz', type=str, required=False, default=None, help="Dump data into supplied NPZ file")

    ap.add_argument('--start', type=float, required=False, default=50e6, help="Start frequency in Hz (default: 50 MHz)")
    ap.add_argument('--end', type=float, required=False, default=4400e6, help="End frequency in Hz (default: 4.4 GHz)")
    ap.add_argument('--step', type=float, required=False, default=1e3, help="Step size in Hz (default: 1 kHz)")

    args = ap.parse_args()

    return args

def main():
    # Parse arguments
    args = _parseArguments()

    if ((args.start < 50e6) or (args.start > 4400e6) or (args.end < 50e6) or (args.end > 4400e6)):
        print(f"Start frequency {args.start} or end frequency {args.end} is out of supported range from 50 MHz to 4.4 GHz")
        sys.exit(1)
    if (args.start >= args.end):
        print(f"Start frequency has to be smaller than end frequency")
        sys.exit(1)
    if (args.step < 1e3) or (args.step > 10e6):
        print("Step size has to be in range of 1 kHz to 10 MHz")
        sys.exit(1)

    plotting = False
    if args.plot != None:
        plotting = True
    if args.show:
        plotting = True

    if args.debug:
        print(f"Plotting: {plotting}")

    if not args.s00 and not args.s01:
        print(f"You have to select at least --s00 or --s01")
        sys.exit(1)

    if plotting:
        import matplotlib.pyplot as plt

    with NanoVNAV2(args.port, debug = args.debug, useNumpy = True) as vna:
        _id = vna._get_id()
        if args.debug:
            print(f"NanoVNA v2 identified as {_id}")

        if args.debug:
            print(f"Setting:")
            print(f"\tStart frequency {args.start}")
            print(f"\tEnd frequency   {args.end}")
            print(f"\tStep size       {args.step}")

        vna._set_sweep_range(args.start, args.end, args.step)
        if args.debug:
            print("Querying trace ...")

        data = vna._query_trace()

        fig, ax = None, None
        if plotting:
            if args.phases:
                fig, ax = plt.subplots(2, figsize=(6.4, 4.8*2))

                if args.s00:
                    ax[0].plot(data["freq"]/1e6, data["s00rawdbm"], label=args.label00)
                if args.s01:
                    ax[0].plot(data["freq"]/1e6, data["s01rawdbm"], label=args.label01)
                ax[0].set_xlabel("Frequency [MHz]")
                ax[0].set_ylabel("Power [dB]")
                ax[0].set_title(args.plottitle)
                ax[0].grid()
                ax[0].legend()

                if args.s00:
                    ax[1].plot(data["freq"]/1e6, data["s00rawphase"], label=args.label00)
                if args.s01:
                    ax[1].plot(data["freq"]/1e6, data["s01rawphase"], label=args.label01)
                ax[1].set_xlabel("Frequency [MHz]")
                ax[1].set_ylabel("Phase [rad]")
                ax[1].grid()
                ax[1].legend()
            else:
                fig, ax = plt.subplots()

                if args.s00:
                    ax.plot(data["freq"]/1e6, data["s00rawdbm"], label=args.label00)
                if args.s01:
                    ax.plot(data["freq"]/1e6, data["s01rawdbm"], label=args.label01)
                ax.set_xlabel("Frequency [MHz]")
                ax.set_ylabel("Power [dB]")
                ax.set_title(args.plottitle)
                ax.grid()
                ax.legend()

        if args.npz:
            np.savez(args.npz, **data)

        if args.plot:
            for fn in args.plot:
                if args.debug:
                    print(f"Saving {fn[0]}")
                plt.savefig(fn[0])

        if args.show:
            plt.show()

if __name__ == "__main__":
    main()
