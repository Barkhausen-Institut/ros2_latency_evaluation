from typing import List
from glob import glob
import os

def createResultsFilepath(args, fileExtension: str) -> str:
    filename = f"{args.rmw}_"
    if len(args.f) > 1:
        filename += f"{args.nodes[0]}Nodes_"
        for f in args.f:
            filename += f"{f}-"
    else:
        filename += f"{args.nodes[0]}-{args.nodes[-1]}Nodes_{args.f[0]}"

    filename += f"Hz_{args.msg_size}_{args.reliability}.{fileExtension}"
    filePath = os.path.join(args.res_dir, filename)
    return filePath

def getRelevantDirectories(args) -> List[str]:
    """Gets relevant directories provided the parameters in args.

    Args are the parsed arguments of the argparser, consisting of the following parameters:
    - f:
    - msg_size:
    - rmw
    - nodes
    - reliability
    """
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