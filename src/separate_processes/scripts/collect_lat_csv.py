import argparse
import os
from glob import glob
from typing import List


def getRelevantDirectories(args) -> List[str]:
    dirPaths: List[str] = []
    for noNodes in range(args.nodes[0], args.nodes[1] + 1, args.nodes[2]):
        globPattern = f"*{noNodes}Nodes*{args.f}Hz*{args.msg_size}*{args.rmw}*{args.reliability}*"
        dirPaths.extend(glob(os.path.join(args.directory, globPattern)))

    if len(dirPaths) == 0:
        raise FileNotFoundError("No directories found.")
    else:
        print(f"Processing following directories: {dirPaths}")

    return dirPaths
def processDirectory(args) -> None:
    if not os.path.exists(args.directory):
        raise FileNotFoundError(f"Directory {parentDir} does not exist.")

    dirPaths = getRelevantDirectories(args)

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
    processDirectory(args)