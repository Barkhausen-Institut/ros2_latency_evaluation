import argparse
import os
from glob import glob
from typing import List, Tuple, Any
import json

import pandas as pd

from utils import getRelevantDirectories, createResultsFilepath

def replaceEmptyListsByZeroElements(l: List[List[Any]]) -> List[Any]:
    res = []
    for subList in l:
        if len(subList) == 0:
            res.extend([0])
        else:
            res.extend(subList)
    return res

def loadStats(desiredArgument, dirPaths: List[str]) -> pd.DataFrame:
    with open(os.path.join(dirPaths[0], "stats.json"), 'r') as f:
        stats = json.load(f)
        statsDf = pd.DataFrame(index=desiredArgument, columns=stats.keys())
    for arg, dirPath in zip(desiredArgument, dirPaths):
        with open(os.path.join(dirPath, "stats.json"), 'r') as f:
            stats = json.load(f)
            for k in stats.keys():
                if k == "invalidMsgs":
                    indices = [v for _, v in stats[k].items()]
                    statsDf.loc[arg, k] = replaceEmptyListsByZeroElements(indices)
                else:
                    statsDf.loc[arg, k] = stats[k]

    return statsDf

def processDirectory(args) -> pd.DataFrame:
    if not os.path.exists(args.directory):
        raise FileNotFoundError(f"Directory {parentDir} does not exist.")

    print(f"Parsing directory: {args.directory}")
    dirPaths = getRelevantDirectories(args)

    if len(args.f) > 1:
        return loadStats(args.f, dirPaths)
    else:
        return loadStats(args.nodes, dirPaths)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', type=str, default=".", help='relative path to parent directory containing dumped csvs.')
    parser.add_argument('--res-dir', type=str, default="results_paper", help="Directory to save post-processed data to.")
    parser.add_argument('--nodes', nargs="+", default=[n for n in range(3, 24, 2)], type=int,
                                   help="""Number of nodes to process. Pass it as follows:
                                            <start_nodes> <end_nodes> <step_size>.""")
    parser.add_argument('--rmw', type=str, default="fastrtps", help="Choose RMW. Allowed values: cyclone, fastrtps, connext")
    parser.add_argument('--f', nargs="+", default=[1], type=int, help="Publisher Frequency in Hz.")
    parser.add_argument('--msg-size', type=str, default="128b", help="Size of the message.")
    parser.add_argument('--reliability', type=str, default="reliable", help="Reliability, best-effort or reliable")
    args = parser.parse_args()

    statsDf = processDirectory(args)
    filePath, _ = createResultsFilepath(args, "csv")

    if not os.path.exists(args.res_dir):
        os.makedirs(args.res_dir)

    if len(args.f) > 1:
        statsDf.to_csv(path_or_buf=filePath, sep=',', na_rep="nan", index_label="F[Hz]")
    else:
        statsDf.to_csv(path_or_buf=filePath, sep=',', na_rep="nan", index_label="Nodes")

    print(f"Results saved to: {os.path.abspath(filePath)}")
