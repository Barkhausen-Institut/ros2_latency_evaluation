import os
import argparse
from glob import glob
from typing import List, Tuple
import csv
import numpy as np

PROF_IDX_LABELS_MAPPING = [
        "prof_PUB_RCLCPP_INTERPROCESS_PUBLISH 0",
        "prof_PUB_RCL_PUBLISH",
        "prof_PUB_RMW_PUBLISH",
        "prof_PUB_DDS_WRITE",
        "prof_SUB_DDS_ONDATA",
        "prof_SUB_RCLCPP_TAKE_ENTER",
        "prof_SUB_RCL_TAKE_ENTER",
        "prof_SUB_RMW_TAKE_ENTER",
        "prof_SUB_DDS_TAKE_ENTER",
        "prof_SUB_DDS_TAKE_LEAVE",
        "prof_SUB_RMW_TAKE_LEAVE",
        "prof_SUB_RCL_TAKE_LEAVE",
        "prof_SUB_RCLCPP_TAKE_LEAVE",
        "prof_SUB_RCLCPP_HANDLE"
    ]

NO_PROFILING_TIMESTAMPS = len(PROF_IDX_LABELS_MAPPING)

def getNodeIndexFromDumpedCsvFileName(filename: str) -> int:
    nodeIdx = int(filename.split('-')[0])
    return nodeIdx

def getNoNodesFromDumpedCsvFileName(filename: str) -> int:
    noNodes = filename.split('-')[1]
    noNodes = int(noNodes[:-4])
    return noNodes

def sortCsvFiles(csvFiles: List[str]):
    filesBasenames = [os.path.basename(f) for f in csvFiles]
    unsortedFiles = {}
    for basePath, completePath in zip(filesBasenames, csvFiles):
        unsortedFiles[getNodeIndexFromDumpedCsvFileName(basePath)] = completePath

    sortedFiles = {k: unsortedFiles[k] for k in sorted(unsortedFiles)}
    del sortedFiles[1]
    return sortedFiles

def findDroppedMsgs(trackingNumbers: List[np.array]) -> np.array:
    breakpoint()
    trackingNumbersDropped = [np.array([]) for msgIdx in range(len(trackingNumbers)-1)]
    firstMsg_trackingNumbers = trackingNumbers[0]
    msgs_trackingNumbers = trackingNumbers[1:]

    for trackingNumber in firstMsg_trackingNumbers:
        for msgIdx, msg in enumerate(trackingNumbers[1:]):
            for i, t in enumerate(msg):
                if not (t == trackingNumber) and (i == len(msg) - 1):
                    trackingNumbersDropped[msgIdx] = np.append(
                        trackingNumbersDropped[msgIdx], trackingNumber
                    )
                elif (t == trackingNumber):
                    break
    trackingNumbersDropped = np.concatenate(trackingNumbersDropped)
    print(f"Messages with following tracking numbers are to be deleted: {trackingNumbersDropped}.")
    return trackingNumbersDropped

def removeDroppedMsgs(csvContents: List[np.array], trackingNumbers: np.array) -> List[np.array]:
    return csvContents

def readCsvs(sortedCsvs) -> Tuple[List[str], List[np.array]]:
    with open(sortedCsvs[2]) as f:
        header = f.readline().split(',')

    csvContents = []
    for nodeIdx, filePath in sortedCsvs.items():
        csvContents.append(np.genfromtxt(filePath, delimiter=",", skip_header=1))
    trackingNumbers = [content[:, 0] for content in csvContents]
    trackingNumbers_droppedMsgs = findDroppedMsgs(trackingNumbers)
    csvContents = removeDroppedMsgs(csvContents, trackingNumbers_droppedMsgs)
    return header, csvContents

def calcLatenciesEndToEnd(parentDir: str):
    if not os.path.exists(parentDir):
        raise FileNotFoundError(f"Directory {parentDir} does not exist.")
    dumpedCsvsPerRun = glob(f"{parentDir}/[0-9]*-[0-9]*.csv")
    sortedCsvs = sortCsvFiles(dumpedCsvsPerRun)
    noNodes = getNoNodesFromDumpedCsvFileName(os.path.basename(dumpedCsvsPerRun[0]))

    header, csvContents = readCsvs(sortedCsvs)
    NO_SAMPLES = 10000
    latenciesProfiling = np.zeros((NO_SAMPLES, NO_PROFILING_TIMESTAMPS))

    for msg in csvContents:
        for profIdx in range(1, NO_PROFILING_TIMESTAMPS):
            latenciesProfiling[:, profIdx+1] += msg[:, profIdx+1] - msg[:, profIdx]

        latenciesProfiling[:, 0] += msg[:, 1] - msg[:, 0]
    return NO_SAMPLES, latenciesProfiling

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=str, help='relative path to directory containing dumped csvs.')
    args = parser.parse_args()

    for resultsDir in glob(os.path.join(args.directory, "*")):
        print(f"Parsing directory: {resultsDir}")
        noSamples, latencies = calcLatenciesEndToEnd(resultsDir)
        with open(os.path.join(resultsDir, "latencies.csv"), "w") as f:
            writer = csv.DictWriter(f, fieldnames=latencies.keys())
            writer.writeheader()

            for sample in range(noSamples):
                data = {}
                for k in latencies.keys():
                    data[k] = latencies[k][sample]
                writer.writerow(data)
