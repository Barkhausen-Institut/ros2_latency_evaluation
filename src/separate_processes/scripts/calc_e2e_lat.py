import os
import argparse
from glob import glob
from typing import List
import csv


def getNodeIndexFromDumpedCsvFileName(filename: str) -> int:
    nodeIdx = int(filename.split('-')[0])
    return nodeIdx

def sortCsvFiles(csvFiles: List[str]):
    filesBasenames = [os.path.basename(f) for f in csvFiles]
    unsortedFiles = {}
    for basePath, completePath in zip(filesBasenames, csvFiles):
        unsortedFiles[getNodeIndexFromDumpedCsvFileName(basePath)] = completePath

    sortedFiles = {k: unsortedFiles[k] for k in sorted(unsortedFiles)}
    return sortedFiles

parser = argparse.ArgumentParser()
parser.add_argument('directory', type=str, help='relative path to directory containing dumped csvs.')
args = parser.parse_args()

if not os.path.exists(args.directory):
    raise FileNotFoundError(f"Directory {args.directory} does not exist.")

dumpedCsvsPerRun = glob(f"{args.directory}/*")
sortedCsvs = sortCsvFiles(dumpedCsvsPerRun)
print(sortedCsvs)


