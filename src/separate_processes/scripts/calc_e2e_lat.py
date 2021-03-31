import os
import argparse
from glob import glob
from typing import List
import csv
import numpy as np

NO_PROFILING_TIMESTAMPS = 14
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

def findDroppedMsgs(trackingNumbers: List[np.array]):
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

def readCsvs(sortedCsvs):
    trackingNumbers = []
    minNoSamples = float("inf")
    maxNoSamples = -1

    timestampHeaders = ["header_timestamp"] + [PROF_IDX_LABELS_MAPPING[i] for i in range(NO_PROFILING_TIMESTAMPS)] + ["callback_timestamp"]
    timestamps = {}
    for nodeIdx, filePath in sortedCsvs.items():
        noSamples = 0
        timestampsCurrFile = {k: [] for k in timestampHeaders}
        currTrackingNumbers = np.array([])
        with open(filePath) as f:
            reader = csv.DictReader(f, delimiter=',')
            for row in reader:
                for header in timestampHeaders:
                    timestampsCurrFile[header].append(float(row[header]))
                currTrackingNumbers = np.append(currTrackingNumbers, int(row["tracking_number"]))

            timestamps[nodeIdx] = timestampsCurrFile
        trackingNumbers.append(currTrackingNumbers)

        minNoSamples = min(noSamples, minNoSamples)
        maxNoSamples = max(noSamples, maxNoSamples)
        if not (minNoSamples == maxNoSamples):
            print("WARNING: Varying amount of samples.")
            print(f"Encountered for Node idx {nodeIdx} and file {filePath}")
            print("Using minNoSamples")

    discardDroppedMsgs(trackingNumbers)
    return minNoSamples, timestamps

def calcLatenciesEndToEnd(parentDir: str):
    if not os.path.exists(parentDir):
        raise FileNotFoundError(f"Directory {parentDir} does not exist.")
    dumpedCsvsPerRun = glob(f"{parentDir}/[0-9]*-[0-9]*.csv")
    sortedCsvs = sortCsvFiles(dumpedCsvsPerRun)
    noNodes = getNoNodesFromDumpedCsvFileName(os.path.basename(dumpedCsvsPerRun[0]))

    noSamples, timestamps = readCsvs(sortedCsvs)

    latencies = {"e2e": np.zeros(noSamples)}
    for i in range(NO_PROFILING_TIMESTAMPS):
        latencies[PROF_IDX_LABELS_MAPPING[i]] = np.zeros(noSamples)

    for nodeIdx in timestamps.keys():
        for i in range(NO_PROFILING_TIMESTAMPS-2):
            currProfilingTimestamps = timestamps[nodeIdx][PROF_IDX_LABELS_MAPPING[i]]
            nextProfilingTimestamps = timestamps[nodeIdx][PROF_IDX_LABELS_MAPPING[i+1]]
            latencies[PROF_IDX_LABELS_MAPPING[i+1]] += np.array(nextProfilingTimestamps) - np.array(currProfilingTimestamps)

        latencies[PROF_IDX_LABELS_MAPPING[0]] += np.array(timestamps[nodeIdx][PROF_IDX_LABELS_MAPPING[0]]) - np.array(timestamps[nodeIdx]["header_timestamp"])
        latencies[PROF_IDX_LABELS_MAPPING[13]] += np.array(timestamps[nodeIdx][PROF_IDX_LABELS_MAPPING[13]]) - np.array(timestamps[nodeIdx][PROF_IDX_LABELS_MAPPING[i]])
    latencies["e2e"] = np.array(timestamps[noNodes]["callback_timestamp"]) - np.array(timestamps[2]["header_timestamp"])
    return noSamples, latencies

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
