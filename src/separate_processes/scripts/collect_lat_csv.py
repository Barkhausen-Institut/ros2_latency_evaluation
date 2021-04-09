import argparse
import os
from glob import glob
from typing import List
import json

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
    for noNodes in range(args.nodes[0], args.nodes[1] + 1, args.nodes[2]):
        globPattern = f"*_{noNodes}Nodes*{args.f}Hz*{args.msg_size}*{args.rmw}*{args.reliability}*"
        dirPaths.extend(glob(os.path.join(args.directory, globPattern)))

    if len(dirPaths) == 0:
        raise FileNotFoundError("No directories found.")
    else:
        print(f"Processing following directories: {dirPaths}")

    return dirPaths

def processDirectory(args) -> None:
    resStats: Dict[str, List[float]] = {k: [] for k in PROF_CATEGORIES.keys()}
    if not os.path.exists(args.directory):
        raise FileNotFoundError(f"Directory {parentDir} does not exist.")

    dirPaths = getRelevantDirectories(args)
    for dirPath in dirPaths:
        with open(os.path.join(dirPath, "stats.json"), 'r') as f:
            stats = json.load(f)
            for profCategory in PROF_CATEGORIES.keys():
                resStats[profCategory].append(stats[f"{profCategory}_{args.stat_quantity}"])

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
    parser.add_argument('--stat-quantity', type=str, default="median", help="Statistical quanitity to choose.")
    args = parser.parse_args()

    print(f"Parsing directory: {args.directory}")
    processDirectory(args)