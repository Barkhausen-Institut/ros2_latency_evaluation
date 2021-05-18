import numpy as np
from typing import Tuple, List
import csv
import matplotlib.pyplot as plt
import pandas as pd
from glob import glob
import os
import argparse
import json


from utils import getRelevantDirectories, createResultsFilepath

def calculateCdf(lat: np.array) -> Tuple[np.array, np.array]:
    binCount, binEdges = np.histogram(lat, bins=100)
    pdf = binCount / sum(binCount)
    cdf = np.cumsum(pdf)

    return binEdges, cdf


def calculateAggPkgErrors(pkgs: np.array) -> np.array:
    errors = np.diff(pkgs) - 1
    aggErrors = np.cumsum(errors) + pkgs[0]
    aggErrors = np.append(np.array([0]), aggErrors)
    return aggErrors


def processDirectory(args, evalType: str):
    plt.ion()
    dirs: List[str] = getRelevantDirectories(args)
    statsLatFound = False

    filePath, filenameAppendix = createResultsFilepath(args, "png")
    for f in dirs:
        if not os.path.exists(os.path.join(f, 'stats.json')) or not os.path.exists(os.path.join(f,'latencies.csv')):
            print(f"{f} does not contain a stats.json or latencies.csv")
        else:
            statsLatFound = True
            latenciesFile = os.path.join(f, "latencies.csv")
            print(f"Processing file: {latenciesFile}")

            df = pd.read_csv(latenciesFile)
            stats = json.load(open(os.path.join(f, 'stats.json'), 'r'))
            lenInvalidMsgs = len(stats['invalidMsgs'].keys())

            title = f'RMW: {args.rmw}, Freq: {args.f}, Msgsize: {args.msg_size}, Rel.: {args.reliability}'

            plt.title(title)
            if evalType == "pkg_errors":
                trackingNumbers = df["tracking_number"].values
                aggregatedErrors = calculateAggPkgErrors(trackingNumbers)
                plt.xlabel('Tracking Number [#]')
                plt.ylabel('Aggregated Msg Drop')
                plt.plot(np.arange(len(aggregatedErrors)), aggregatedErrors, label=f'len(invalidMsgs)={lenInvalidMsgs}')

                filename = "aggregatedMsgDrop_"

            if evalType == "cdf":
                endToEndLatencies = df["end2end"].values
                binEdges, cdf = calculateCdf(endToEndLatencies)

                plt.plot(binEdges[1:], cdf, label=f'len(noInvalidMsgs)={lenInvalidMsgs}')
                plt.xlabel('Latencies [us]')
                plt.ylabel('Probability')
                plt.yticks([0.1*i for i in range(11)])
                plt.grid('minor')

                filename = "e2eLatCdf_"

            plt.legend()
            filename += filenameAppendix

    if not os.path.exists(args.res_dir):
        os.makedirs(args.res_dir)
    if statsLatFound:
        plt.savefig(os.path.join(args.res_dir, filename))
    plt.clf()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', type=str, default=".", help='relative path to parent directory containing dumped csvs.')
    parser.add_argument('--res-dir', type=str, default="results_paper", help="Directory to save post-processed data to.")
    parser.add_argument('--nodes', nargs="+", default=[n for n in range(3, 26, 2)], type=int,
                                   help="""Number of nodes to process. Pass it as follows:
                                            <start_nodes> <end_nodes> <step_size>.""")
    parser.add_argument('--rmw', type=str, default="fastrtps", help="Choose RMW. Allowed values: cyclone, fastrtps, connext")
    parser.add_argument('--f', nargs="+", default=[1], type=int, help="Publisher Frequency in Hz.")
    parser.add_argument('--msg-size', type=str, default="128b", help="Size of the message.")
    parser.add_argument('--reliability', type=str, default="reliable", help="Reliability, best-effort or reliable")
    parser.add_argument('--pkg-errors', type=bool, default=False)
    parser.add_argument('--cdf', type=bool, default=False)
    args = parser.parse_args()

    if args.cdf:
        processDirectory(args, "cdf")
    if args.pkg_errors:
        processDirectory(args, "pkg_errors")



