import numpy as np
from typing import Tuple
import csv
import matplotlib.pyplot as plt
import pandas as pd
from glob import glob
import os
import argparse

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

parser = argparse.ArgumentParser()
parser.add_argument('--pkg-errors', type=bool, default=False)
parser.add_argument('--cdf', type=bool, default=False)
args = parser.parse_args()

parentDir = "/home/kronauer/Documents/06_ICRA_Paper/results/res"
fileList = glob(os.path.join(parentDir, '*', 'latencies.csv'))

plt.ion()
for f in fileList:
    print(f"Processing file: {f}")

    df = pd.read_csv(f)

    if args.pkg_errors:
        trackingNumbers = df["tracking_number"].values
        aggregatedErrors = calculateAggPkgErrors(trackingNumbers)

        plt.xlabel('Tracking Number')
        plt.ylabel('Aggregated Msg Drop')
        plt.plot(np.arange(len(aggregatedErrors)), aggregatedErrors)
        plt.title(f'Lost Messages: {trackingNumbers[-1] - len(trackingNumbers)}')
        plt.draw()

        filename = "aggregatedMsgDrop.png"

    if args.cdf:
        endToEndLatencies = df["end2end"].values
        binEdges, cdf = calculateCdf(endToEndLatencies)

        plt.plot(binEdges[1:], cdf)
        plt.xlabel('Latencies')
        plt.ylabel('Probability')
        plt.yticks([0.1*i for i in range(11)])
        plt.grid('minor')
        plt.draw()
        filename = "e2eLatCdf.png"

    plt.savefig(os.path.join(
        os.path.dirname(f), filename))
    plt.clf()



