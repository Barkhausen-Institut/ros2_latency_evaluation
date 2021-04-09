import argparse
import os
from glob import glob
from typing import List, Tuple
import json

import pandas as pd

# define the profiling categories. The values are pairwise start-end
# profiling index names. Time span between multiple pairs are added up
# within the category. Current setting matches the figure from the current paper diagram.
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


def getRelevantDirectories(args) -> List[str]:
    dirPaths: List[str] = []
    nodes: List[int] = []

    for noNodes in range(args.nodes[0], args.nodes[1] + 1, args.nodes[2]):
        globPattern = f"*_{noNodes}Nodes*{args.f}Hz*{args.msg_size}*{args.rmw}*{args.reliability}*"
        globbedDirectories = glob(os.path.join(args.directory, globPattern))

        # only update if some directories are actually found
        if len(globbedDirectories) > 1:
            raise ValueError("We foundmore than one directory...")

        if len(globbedDirectories) == 1:
            dirPaths.append(globbedDirectories[0])
            nodes.append(noNodes)

    if len(dirPaths) == 0:
        raise FileNotFoundError("No directories found.")

    return nodes, dirPaths

def createResultsFilepath(args) -> str:
    filename = f"{args.rmw}_{args.f}Hz_{args.msg_size}_{args.reliability}.csv"
    filePath = os.path.join(args.res_dir, filename)
    return filePath

def processDirectory(args) -> pd.DataFrame:
    if not os.path.exists(args.directory):
        raise FileNotFoundError(f"Directory {parentDir} does not exist.")

    nodes, dirPaths = getRelevantDirectories(args)
    with open(os.path.join(dirPaths[0], "stats.json"), 'r') as f:
        stats = json.load(f)
        statsDf = pd.DataFrame(index=nodes, columns=stats.keys(), dtype=float)

    for noNodes, dirPath in zip(nodes, dirPaths):
        with open(os.path.join(dirPath, "stats.json"), 'r') as f:
            stats = json.load(f)
            for k in stats.keys():
                statsDf.loc[noNodes, k] = stats[k]
    return statsDf

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', type=str, default=".", help='relative path to parent directory containing dumped csvs.')
    parser.add_argument('--res-dir', type=str, default="results_paper", help="Directory to save post-processed data to.")
    parser.add_argument('--nodes', nargs="+", default=[3, 23, 1], help="""Number of nodes to process. Pass it as follows:
                                                                          <start_nodes> <end_nodes> <step_size>.""")
    parser.add_argument('--rmw', type=str, default="fastrtps", help="Choose RMW. Allowed values: cyclone, fastrtps, connext")
    parser.add_argument('--f', type=int, default=1, help="Publisher Frequency in Hz.")
    parser.add_argument('--msg-size', type=str, default="128b", help="Size of the message.")
    parser.add_argument('--reliability', type=str, default="reliable", help="Reliability, best-effort or reliable")
    args = parser.parse_args()

    print(f"Parsing directory: {args.directory}")
    statsDf = processDirectory(args)
    filePath = createResultsFilepath(args)

    if not os.path.exists(args.res_dir):
        os.makedirs(args.res_dir)

    statsDf.to_csv(path_or_buf=filePath, sep=',', na_rep="nan", index_label="Nodes")
    print(f"Results saved to: {os.path.abspath(filePath)}")