import unittest
import csv
import numpy as np
import os

from scripts.calc_e2e_lat import calcLatenciesEndToEnd

class TestE2eLat(unittest.TestCase):

    def setUp(self) -> None:
        self._createTestFiles()
        self.calculatedLatencies = calcLatenciesEndToEnd('.')

    def _createTestFiles(self):
        self.NO_PROFILING_TIMESTAMPS = 14
        HEADERS = ["header_timestamp"] + [f"prof_{i}" for i in range(self.NO_PROFILING_TIMESTAMPS)] + ["callback_timestamp"]
        self.T_MSG_SENT = [0, 1e3, 2e3]
        NO_MSGS = len(self.T_MSG_SENT)
        DELAY_INTER_NODE = 5

        self.t_secondNode = [dict.fromkeys(HEADERS) for i in range(NO_MSGS)]
        self.t_thirdNode = [dict.fromkeys(HEADERS) for i in range(NO_MSGS)]
        self.PROFILING_DELAYS = {f"prof_{i}": np.zeros(len(self.T_MSG_SENT)) for i in range(self.NO_PROFILING_TIMESTAMPS)}

        for i in range(self.NO_PROFILING_TIMESTAMPS):
            self.PROFILING_DELAYS[f"prof_{i}"] = np.random.randint(low=10, high=15, size=NO_MSGS)

        for msgCount, tMsgSent in enumerate(self.T_MSG_SENT):
            # fill up second node profiling
            self.t_secondNode[msgCount]["header_timestamp"] = tMsgSent
            self.t_secondNode[msgCount]["prof_0"] = tMsgSent + self.PROFILING_DELAYS["prof_0"][msgCount]
            for i in range(1, self.NO_PROFILING_TIMESTAMPS):
                self.t_secondNode[msgCount][f"prof_{i}"] = (
                    self.t_secondNode[msgCount][f"prof_{i-1}"]
                    + self.PROFILING_DELAYS[f"prof_{i}"][msgCount]
                )
            self.t_secondNode[msgCount]["callback_timestamp"] = self.t_secondNode[msgCount]["prof_13"] + 20

            # fill up third node profiling
            self.t_thirdNode[msgCount]["header_timestamp"] = tMsgSent
            self.t_thirdNode[msgCount]["prof_0"] = tMsgSent + self.PROFILING_DELAYS["prof_0"][msgCount]
            for i in range(1, self.NO_PROFILING_TIMESTAMPS):
                self.t_thirdNode[msgCount][f"prof_{i}"] = (
                    self.t_thirdNode[msgCount][f"prof_{i-1}"]
                    + self.PROFILING_DELAYS[f"prof_{i}"][msgCount]
                )
            self.t_thirdNode[msgCount]["callback_timestamp"] = self.t_thirdNode[msgCount]["prof_13"] + 20

        with open('0-3.csv', 'w') as f:
            writer = csv.DictWriter(f, fieldnames=HEADERS)
            writer.writeheader()

        with open('1-3.csv', 'w') as f:
            writer = csv.DictWriter(f, fieldnames=HEADERS)
            writer.writeheader()
            writer.writerows(self.t_secondNode)

        with open('2-3.csv', 'w') as f:
            writer = csv.DictWriter(f, fieldnames=HEADERS)
            writer.writeheader()
            writer.writerows(self.t_thirdNode)
    
    def tearDown(self) -> None:
        os.remove('0-3.csv')
        os.remove('1-3.csv')
        os.remove('2-3.csv')

    def test_correctEToELatency(self) -> None:
        e2eLatencies = np.zeros(len(self.T_MSG_SENT))
        for i in range(len(self.T_MSG_SENT)):
            e2eLatencies[i] = self.t_thirdNode[i]["callback_timestamp"] - self.t_secondNode[i]["header_timestamp"]

        np.testing.assert_array_almost_equal(
            self.calculatedLatencies["e2e"],
            e2eLatencies
        )

    def test_correctProfilingEToELatency(self) -> None:
        for k in self.PROFILING_DELAYS.keys():
            print(f"Evaluating: {k}")
            np.testing.assert_array_almost_equal(
                self.calculatedLatencies[k],
                self.PROFILING_DELAYS[k]*2
            )