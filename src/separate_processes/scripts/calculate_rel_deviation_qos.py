import argparse
import os

import pandas as pd


parser = argparse.ArgumentParser()
parser.add_argument('--rel-csv', type=str)
parser.add_argument('--be-csv', type=str)
parser.add_argument('--res-dir', type=str)
parser.add_argument('--csv-name', type=str)
args = parser.parse_args()

relCsvPath = args.rel_csv
bestEffortCsvPath = args.be_csv
resDir = args.res_dir
csvName = args.csv_name

relDf = pd.read_csv(relCsvPath)
beDf = pd.read_csv(bestEffortCsvPath)

relDev = (100*(relDf.loc[:, "Publisher_ROS2_Common_mean":"noTotalInvalidMsgs"]
            - beDf.loc[:, "Publisher_ROS2_Common_mean" : "noTotalInvalidMsgs"])
            / beDf.loc[:, "Publisher_ROS2_Common_mean":"noTotalInvalidMsgs"])
relDev.insert(loc=0, column='Nodes', value=beDf.loc[:, 'Nodes'])
relDev.set_index("Nodes")

print(f"Saving to: {os.path.join(resDir, csvName)}")
relDev.to_csv(path_or_buf=os.path.join(resDir, csvName),
                sep=',', na_rep="nan", index_label="RelDeviation")