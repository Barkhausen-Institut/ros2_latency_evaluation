import os
import argparse
from glob import glob
from typing import List

def getNodeIndexFromDumpedCsvFileName(filename: str) -> int:
    nodeIdx = int(filename.split('-')[0])
    return nodeIdx

def sortCsvFiles(csvFiles: List[str]):
    unsortedFiles = {}
    for f in csvFiles:
        unsortedFiles[getNodeIndexFromDumpedCsvFileName(f)] = f

    sortedFiles = {k: unsortedFiles[k] for k in sorted(unsortedFiles)}
    return sortedFiles

parser = argparse.ArgumentParser()
parser.add_argument('directory', type=str, help='relative path to directory containing dumped csvs.')
args = parser.parse_args()

if not os.path.exists(args.directory):
    raise FileNotFoundError(f"Directory {args.directory} does not exist.")

dumpedCsvsPerRun = [os.path.basename(f) for f in glob(f"{args.directory}/*")]
sortedCsvs = sortCsvFiles(dumpedCsvsPerRun)

print(f"Processing directory: {args.directory}")
for dumpedCsv in dumpedCsvsPerRun:
    print(f"Current node index: {getNodeIndexFromDumpedCsvFileName(dumpedCsv)}")

