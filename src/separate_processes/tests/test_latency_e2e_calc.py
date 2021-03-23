import unittest
import csv
import numpy as np

class TestE2eLat(unittest.TestCase):
    def _createTestFiles(self):
        NO_PROFILING_TIMESTAMPS = 14
        HEADERS = ["header_timestamp"] + [f"prof_{i}" for i in range(NO_PROFILING_TIMESTAMPS)] + ["callback_timestamp"]
        T_MSG_SENT = [0, 1e3, 2e3]
        DELAY_INTER_NODE = 5

        t_secondNode = [dict.fromkeys(HEADERS) for i in range(len(T_MSG_SENT))]
        t_thirdNode = [dict.fromkeys(HEADERS) for i in range(len(T_MSG_SENT))]

        for msgCount, tMsgSent in enumerate(T_MSG_SENT):
            # fill up second node profiling
            t_secondNode[msgCount]["header_timestamp"] = tMsgSent
            for i in range(NO_PROFILING_TIMESTAMPS):
                t_secondNode[msgCount][f"prof_{i}"] = tMsgSent + (i + 1) * 10
            t_secondNode[msgCount]["callback_timestamp"] = t_secondNode[msgCount]["prof_13"] + 20

            # fill up third node profiling
            t_thirdNode[msgCount]["header_timestamp"] = tMsgSent
            for i in range(NO_PROFILING_TIMESTAMPS):
                t_thirdNode[msgCount][f"prof_{i}"] = tMsgSent + (i + 1) * 10 + DELAY_INTER_NODE
            t_thirdNode[msgCount]["callback_timestamp"] = t_thirdNode[msgCount]["prof_13"] + 20

        with open('1-3.csv', 'w') as f:
            writer = csv.DictWriter(f, fieldnames=HEADERS)
            writer.writeheader()
            writer.writerows(t_secondNode)

        with open('2-3.csv', 'w') as f:
            writer = csv.DictWriter(f, fieldnames=HEADERS)
            writer.writeheader()
            writer.writerows(t_thirdNode)