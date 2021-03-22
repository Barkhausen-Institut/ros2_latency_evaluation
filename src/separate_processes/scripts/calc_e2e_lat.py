import os
import argparse
from glob import glob

def getNodeIndexFromDumpedCsvFileName(filename: str):
    nodeIdx = int(filename.split('-')[0])
    return nodeIdx

parser = argparse.ArgumentParser()
parser.add_argument('directory', type=str, help='relative path to directory containing dumped csvs.')
args = parser.parse_args()

if not os.path.exists(args.directory):
    raise FileNotFoundError(f"Directory {args.directory} does not exist.")

dumpedCsvsPerRun = [os.path.basename(f) for f in glob(f"{args.directory}/*")]

print(f"Processing directory: {args.directory}")
for dumpedCsv in dumpedCsvsPerRun:
    print(f"Current node index: {getNodeIndexFromDumpedCsvFileName(dumpedCsv)}")

