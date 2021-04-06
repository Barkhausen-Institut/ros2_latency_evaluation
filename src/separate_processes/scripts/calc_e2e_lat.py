import os
import argparse
from glob import glob
from typing import List, Tuple
import csv
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

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

PROF_CATEGORIES = {
    'Publisher ROS2 Common': ('prof_PUB_RCLCPP_INTERPROCESS_PUBLISH 0',
                              'prof_PUB_RMW_PUBLISH'),
    'Publisher rmw': ('prof_PUB_RMW_PUBLISH',
                      'prof_PUB_DDS_WRITE'),
    'DDS': ('prof_PUB_DDS_WRITE', 'prof_SUB_DDS_ONDATA',
            'prof_SUB_DDS_TAKE_ENTER', 'prof_SUB_DDS_TAKE_LEAVE'),
    'Rclcpp Notification Delay': ('prof_SUB_DDS_ONDATA', 'prof_SUB_RCLCPP_TAKE_ENTER'),
    'Subscriber ROS2 Common': ('prof_SUB_RCLCPP_TAKE_ENTER', 'prof_SUB_RMW_TAKE_ENTER',
                               'prof_SUB_RMW_TAKE_LEAVE', 'prof_SUB_RCLCPP_HANDLE'),
    'Subscriber rmw': ('prof_SUB_RMW_TAKE_ENTER', 'prof_SUB_DDS_TAKE_ENTER',
                       'prof_SUB_DDS_TAKE_LEAVE', 'prof_SUB_RMW_TAKE_LEAVE')
}

def getNodeIndexFromDumpedCsvFileName(filename: str) -> int:
    nodeIdx = int(filename.split('-')[0])
    return nodeIdx

def getNoNodesFromDumpedCsvFileName(filename: str) -> int:
    noNodes = filename.split('-')[1]
    noNodes = int(noNodes[:-4])
    return noNodes

def loadCsvs(files: List[str]):
    header = None
    contents = []
    for fn in files:
        print(f"File {fn}")
        content = pd.read_csv(fn, dtype=np.int64)
        currentHeader = list(content.columns)
        if header is None:
            header = currentHeader
        else:
            assert header == currentHeader
        contents.append(content)
    return contents

def getSortedNamesInDir(parentDir):
    dumpedCsvsPerRun = glob(f"{parentDir}/[0-9]*-[0-9]*.csv")
    noNodes = getNoNodesFromDumpedCsvFileName(os.path.basename(dumpedCsvsPerRun[0]))
    sortedNames = [f"{parentDir}/{i}-{noNodes}.csv" for i in range(2, noNodes+1)]
    for f in sortedNames:
        assert os.path.isfile(f), f"{f} does not exist"
    return sortedNames



def findValidMsgs(csvContents):
    history = []
    validMsgs = None
    startMsgs = None
    for i, content in enumerate(csvContents):
        currentMsgs = set(content['tracking_number'])
        if validMsgs is None:
            startMsgs = currentMsgs
            validMsgs = currentMsgs
        else:
            validMsgs = validMsgs & currentMsgs
        print(f"Dropped in file {i}: {startMsgs - validMsgs}")
        history.append(len(validMsgs))
    return validMsgs, history

def extractValidMsgs(csvContents):
    validMsgs, history = findValidMsgs(csvContents)
    if False:
        plt.plot(history, '-x')
        plt.show()

    result =  [c[c['tracking_number'].isin(validMsgs)] for c in csvContents]
    idx = result[0].index
    for r in result:
        r.set_index(idx, inplace=True)
    def all_equal(iterator):
        iterator = iter(iterator)
        try:
            first = next(iterator)
        except StopIteration:
            return True
        return all(first == x for x in iterator)
    assert all_equal([list(r['tracking_number']) for r in result])
    assert all_equal([list(r.index) for r in result])

    return result

def calcLatencies(content):
    def end2end():
        last = content[-1]
        lastColumn = PROF_IDX_LABELS_MAPPING[-1]
        latencies = last['callback_timestamp'] - last['header_timestamp']
        return latencies

    def category(name, columnNames):
        total = 0
        result = pd.DataFrame(index=content[0].index)
        for i, f in enumerate(content):
            thisContent = 0
            for start, end in zip(columnNames[0::2], columnNames[1::2]):
                thisContent = thisContent + f[end] - f[start]
            total += thisContent
            result[f"{name}_{i+2}"] = thisContent
        result[name] = total
        return result

    result = pd.DataFrame({'tracking_number': content[0]['tracking_number']},
                          index=content[0].index)
    result['end2end'] = end2end()
    for catName, catColumns in PROF_CATEGORIES.items():
        result = result.join(category(catName, catColumns))
    return result

def calcStatistics(latencies):
    pass


def processDirectory(parentDir: str):
    if not os.path.exists(parentDir):
        raise FileNotFoundError(f"Directory {parentDir} does not exist.")
    sortedNames = getSortedNamesInDir(parentDir)
    csvContents = loadCsvs(sortedNames)
    validCsvs = extractValidMsgs(csvContents)

    latencies = calcLatencies(validCsvs)
    latencies.to_csv(f"{parentDir}/latencies.csv", index=False)

    stats = calcStatistics(latencies)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=str, help='relative path to directory containing dumped csvs.')
    args = parser.parse_args()

    for resultsDir in glob(os.path.join(args.directory, "*"))[1:]:
        print(f"Parsing directory: {resultsDir}")
        processDirectory(resultsDir)

        break
