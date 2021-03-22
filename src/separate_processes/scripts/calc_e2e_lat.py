import os
import argparse
from glob import glob
from typing import List
import csv
import numpy as np

def getNodeIndexFromDumpedCsvFileName(filename: str) -> int:
    nodeIdx = int(filename.split('-')[0])
    return nodeIdx

def getNoNodesFromDumpedCsvFileName(filename: str) -> int:
    noNodes = int(filename.split('-')[1][0])
    return noNodes

def sortCsvFiles(csvFiles: List[str]):
    filesBasenames = [os.path.basename(f) for f in csvFiles]
    unsortedFiles = {}
    for basePath, completePath in zip(filesBasenames, csvFiles):
        unsortedFiles[getNodeIndexFromDumpedCsvFileName(basePath)] = completePath

    sortedFiles = {k: unsortedFiles[k] for k in sorted(unsortedFiles)}
    del sortedFiles[0]
    return sortedFiles

parser = argparse.ArgumentParser()
parser.add_argument('directory', type=str, help='relative path to directory containing dumped csvs.')
args = parser.parse_args()

if not os.path.exists(args.directory):
    raise FileNotFoundError(f"Directory {args.directory} does not exist.")

dumpedCsvsPerRun = glob(f"{args.directory}/*")
sortedCsvs = sortCsvFiles(dumpedCsvsPerRun)
noNodes = getNoNodesFromDumpedCsvFileName(os.path.basename(dumpedCsvsPerRun[0]))

timestampHeaders = ["header_timestamp"] + [f"prof_{i}" for i in range(14)] + ["callback_timestamp"]
timestamps = {}

for nodeIdx, filePath in sortedCsvs.items():
    timestampsCurrFile = {k: [] for k in timestampHeaders}
    with open(filePath) as f:
        reader = csv.DictReader(f)
        for row in reader:
            for header in timestampHeaders:
                timestampsCurrFile[header].append(int(row[header]))

        timestamps[nodeIdx] = timestampsCurrFile

latencies = {"e2e": None}

tFirstNode = timestamps[1]['header_timestamp']
tEndNode = timestamps[noNodes - 1]['callback_timestamp']

latencies["e2e"] = np.array(tEndNode) - np.array(tFirstNode)

