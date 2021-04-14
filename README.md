# PingPongComparison

https://redmine.adbi.barkhauseninstitut.org/issues/1405

## Start multiprocesses:
From the directory `src/separate_processes`

```console
$ <parameters> ros2 launch launch/chain.launch.py
```

Check `src/separate_processes/scripts/run_all_benchmarks.sh` for an iteration over all parameters.

##  Overview of parameters

- `INT_NODES=1`: Integer number of nodes to spawn between `start_node` and `end_node`
- `PUB_FREQUENCY=1`: Floating point number of frequency with which the publisher should send its message, in Hz.
- `MSG_SIZE=128b`: Msg size of the topic. Supported values are
  - 128b
  - 1kb
  - 10kb
  - 100kb
  - 500kb
- `QOS=best-effort` : QoS profile. Supported values:
  - reliable
  - best-effort
- `DURATION=10` : Duration of the measurement in seconds.

## Postprocess Results

Run `calc_e2e_lat.py` in `src/separate_processes/scripts/` to calculate the end to end latency + profiling from the first to the last node. Statistics are calculated afterwards. These are saved in `stats.json` in the respective folder.

Afterwards, the you can collect `stats.json` according to your desired parameter configuration:

**collect_lat_csv.py**: Check the argument list. In general, you set the following parameters fix:
- rmw
- msg-size
- reliability

You can either set the number of nodes fix or the frequency while keeping the other variable. "Variable" means that this quantitity is varied according to the list you pass.

Example:

```console
$ python collect_lat_csv.py --directory resultsUnprocessed/ --res-dir results --f 1
```

Frequency is set to 1 and the number of nodes is varied from 3 to 23 with a step size of 2.

However, if you want to see how the latencies varies with the freuqency by having `N` nodes:

```console
$ python collect_lat_csv.py --directory resultsUnprocessed/ --res-dir results --nodes 3 --f 1 10 20
```

In this case, the frequency is varied with the values being `1`, `10`, and `20`.

Results are saved in a `.csv` file in the respective `--res-dir` directory. All stats are column labels, you can pick one of these for e.g. `pgfplots`.

`create_csvs_for_paper.sh` is a convencience script for running multiple parameter configs:

```console
$ bash create_csvs_for_paper.sh <unprocessedResultsDir> <processedResultsDir>
```
