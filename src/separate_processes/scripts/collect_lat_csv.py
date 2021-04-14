import argparse
import os
from glob import glob
from typing import List, Tuple
import json

import pandas as pd

def getRelevantDirectories(args) -> List[str]:
    dirPaths: List[str] = []
    globPatterns: List[str] = []

    if len(args.f) > 1:
        for freq in args.f:
            if freq == 100:
                freq = "1e+02"
            globPattern = f"*_3Nodes*{freq}Hz*{args.msg_size}*{args.rmw}*{args.reliability}*"
            globPatterns.append(globPattern)
    else:
        for noNodes in args.nodes:
            freq = args.f[0]
            if freq == 100:
                freq = "1e+02"

            globPattern = f"*_{noNodes}Nodes*{freq}Hz*{args.msg_size}*{args.rmw}*{args.reliability}*"
            globPatterns.append(globPattern)

    for globPattern in globPatterns:
        globbedDirectories = glob(os.path.join(args.directory, globPattern))
        # only update if some directories are actually found
        if len(globbedDirectories) > 1:
            raise ValueError("We foundmore than one directory...")

        if len(globbedDirectories) == 1:
            dirPaths.append(globbedDirectories[0])

    if len(dirPaths) == 0:
        raise FileNotFoundError("No directories found.")

    return dirPaths

def createResultsFilepath(args) -> str:
    filename = f"{args.rmw}_"
    if len(args.f) > 1:
        filename += f"{args.nodes[0]}Nodes_"
        for f in args.f:
            filename += f"{f}-"
    else:
        filename += f"{args.nodes[0]}-{args.nodes[-1]}Nodes_{args.f[0]}"

    filename += f"Hz_{args.msg_size}_{args.reliability}.csv"
    filePath = os.path.join(args.res_dir, filename)
    return filePath

def loadStats(desiredArgument, dirPaths: List[str]) -> pd.DataFrame:
    with open(os.path.join(dirPaths[0], "stats.json"), 'r') as f:
        stats = json.load(f)
        statsDf = pd.DataFrame(index=desiredArgument, columns=stats.keys(), dtype=float)
    for freq, dirPath in zip(desiredArgument, dirPaths):
        with open(os.path.join(dirPath, "stats.json"), 'r') as f:
            stats = json.load(f)
            for k in stats.keys():
                if k == "invalidMsgs":
                    statsDf.insert(len(statsDf.columns), "amountInvalidMsgs", stats[k]["amount"])
                    statsDf.insert(len(statsDf.columns), "invalidMsgs", pd.Series(stats[k]["indices"]))
                else:
                    statsDf.loc[freq, k] = stats[k]


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
    filePath = createResultsFilepath(args)

    if not os.path.exists(args.res_dir):
        os.makedirs(args.res_dir)

    if len(args.f) > 1:
        statsDf.to_csv(path_or_buf=filePath, sep=',', na_rep="nan", index_label="F[Hz]")
    else:
        statsDf.to_csv(path_or_buf=filePath, sep=',', na_rep="nan", index_label="Nodes")

    print(f"Results saved to: {os.path.abspath(filePath)}")