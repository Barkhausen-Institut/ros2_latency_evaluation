import os
import argparse
from glob import glob
from typing import List, Tuple
import csv
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# set to false to suppress showing result diagrams
SHOW = False

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
    if True:
        plt.figure()
        plt.plot(history, '-x')
        plt.title('Messages going through nodes')
        plt.xlabel('Node index')
        plt.ylabel('Number of messages received until node X')
        plt.grid(True)

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
    sumOverCategories = sum(result[cat] for cat in PROF_CATEGORIES.keys())
    result['sumOverCategories'] = sumOverCategories
    return result

QUANTILES = [0.1, 1, 5, 10, 25, 40, 50, 60, 70, 80, 90, 95, 99, 99.9]

def calcStatistics(latencies):
    def columnStats(name, column):
        result = dict()
        result[f"{name}_mean"] = column.mean()
        result[f"{name}_median"] = column.quantile(0.5)
        for q in QUANTILES:
            result[f"{name}_q{q}"] = column.quantile(q/100.0)
        return result

    columns = list(PROF_CATEGORIES.keys()) + ['end2end', 'sumOverCategories']
    result = dict()
    for c in columns:
        result.update(columnStats(c, latencies[c]))
    return result

def plotStats(stats):
    plt.figure()

    for cat in list(PROF_CATEGORIES.keys()) + ['end2end', 'sumOverCategories']:
        cdf = [stats[f"{cat}_q{q}"] for q in QUANTILES]
        plt.plot(QUANTILES, cdf, '-x', label=cat)
        plt.xlabel('Quantile')
        plt.ylabel('Nanoseconds')
    plt.legend()
    plt.title('Quantile distribution of Latency categories')
    plt.grid(True)
    if SHOW:
        plt.show()


def processDirectory(parentDir: str):
    if not os.path.exists(parentDir):
        raise FileNotFoundError(f"Directory {parentDir} does not exist.")
    sortedNames = getSortedNamesInDir(parentDir)
    csvContents = loadCsvs(sortedNames)
    validCsvs = extractValidMsgs(csvContents)

    latencies = calcLatencies(validCsvs)
    latencies.to_csv(f"{parentDir}/latencies.csv", index=False)

    stats = calcStatistics(latencies)
    print (stats)
    plotStats(stats)
    with open(f"{parentDir}/stats.json", "w") as f:
        import json
        print(json.dumps(stats, indent=4), file=f)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=str, help='relative path to directory containing dumped csvs.')
    args = parser.parse_args()

    for resultsDir in glob(os.path.join(args.directory, "*"))[1:]:
        print(f"Parsing directory: {resultsDir}")
        processDirectory(resultsDir)

        break
