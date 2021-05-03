import numpy as np
from typing import Tuple
import csv
import matplotlib.pyplot as plt
import pandas as pd
from glob import glob
import os
import argparse
import json

from utils import getRelevantDirectories

def calculateCdf(lat: np.array) -> Tuple[np.array, np.array]:
    binCount, binEdges = np.histogram(lat, bins=100)
    pdf = binCount / sum(binCount)
    cdf = np.cumsum(pdf)

    return binEdges, cdf

def calculateAggPkgErrors(pkgs: np.array) -> np.array:
    errors = np.diff(pkgs) - 1
    aggErrors = np.cumsum(errors)
    aggErrors = np.append(np.array([0]), aggErrors)
    return aggErrors

plt.ion()

def processDirectory(args):
    dirs: List[str] = getRelevantDirectories(args)

    for f in getRelevantDirectories(args):
        latenciesFile = os.path.join(f, "latencies.csv")
        print(f"Processing file: {latenciesFile}")

        df = pd.read_csv(latenciesFile)
        stats = json.load(open(os.path.join(f, 'stats.json'), 'r'))
        lenInvalidMsgs = len(stats['invalidMsgs'].keys())
        if args.pkg_errors:
            trackingNumbers = df["tracking_number"].values
            aggregatedErrors = calculateAggPkgErrors(trackingNumbers)
            plt.xlabel('Tracking Number')
            plt.ylabel('Aggregated Msg Drop')
            plt.plot(np.arange(len(aggregatedErrors)), aggregatedErrors, label=f'len(invalidMsgs)={lenInvalidMsgs}')
            plt.legend()

            filename = "aggregatedMsgDrop.png"

        if args.cdf:
            endToEndLatencies = df["end2end"].values
            binEdges, cdf = calculateCdf(endToEndLatencies)

            plt.plot(binEdges[1:], cdf, label=f'len(noInvalidMsgs)={lenInvalidMsgs}')
            plt.xlabel('Latencies')
            plt.ylabel('Probability')
            plt.yticks([0.1*i for i in range(11)])
            plt.grid('minor')
            plt.draw()
            plt.legend()

            filename = "e2eLatCdf.png"

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', type=str, default=".", help='relative path to parent directory containing dumped csvs.')
    parser.add_argument('--res-dir', type=str, default="results_paper", help="Directory to save post-processed data to.")
    parser.add_argument('--nodes', nargs="+", default=[n for n in range(3, 24, 2)], type=int,
                                   help="""Number of nodes to process. Pass it as follows:
                                            <start_nodes> <end_nodes> <step_size>.""")
    parser.add_argument('--rmw', type=str, default="fastrtps", help="Choose RMW. Allowed values: cyclone, fastrtps, connext")
    parser.add_argument('--f', nargs="+", default=[1], type=int, help="Publisher Frequency in Hz.")
    parser.add_argument('--msg-size', type=str, default="128b", help="Size of the message.")
    parser.add_argument('--reliability', type=str, default="reliable", help="Reliability, best-effort or reliable")
    parser.add_argument('--pkg-errors', type=bool, default=False)
    parser.add_argument('--cdf', type=bool, default=False)
    args = parser.parse_args()

    processDirectory(args)


