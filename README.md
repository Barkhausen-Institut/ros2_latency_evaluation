# NodeChainSeparateProcesses

This is the official repository for benchmarking a ros2 node system in separate processses.

In order to see if there are any differences in latency between launching the ros nodes in same process with/without the same context, examples thereof are also included.
Please refer to `src/separate_processes/all_node_same_context.cpp` and `all_nodes_separate_context.cpp`.

In the following, we only describe the scenario with nodes being started in separate processes.

**NOTE: In all subsequent steps we assume that you built our [custom docker image](https://gitlab.com/barkhauseninstitut/corola-papers/ros2latency/customros2foxy).**

# Build

1. `git clone <this-repo>`
2. `docker run -it --name ros2custom --network host -v <path/to/repo>:/ws barkhauseninstitut/ros2custom:foxy20201211`
3. `docker start ros2custom`
4. `docker attach ros2custom`
5. `cd /ws`
6. `. /opt/ros/foxy/setup.bash`
7. `python -m venv env`
8. `colcon build --symlink-install`

# Use

Always source before use:

1. `. /opt/ros/foxy/setup.bash`
2. `. /ws/install/setup.bash`
3. . `/ws/env/bin/activate`

## Start multiprocesses:
From the directory `src/separate_processes/`

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

In the following, we will describe how to get results that are comparable to the paper.

### Calculate End to End latencies from dumped timestamps

Each node dumps in a separate csv absolute timestamps using `CLOCK_MONOTONIC_RAW` upon message receival. Assuming we have `N` nodes in our node chain, the respective csvs are called `<i>-N.csv`, `i` being the 1-based-index of the nodes.

In order to calculate the latencies and profiling from the first node to the `N`-th node, run `calc_e2e_lat.py` in `src/separate_processes/scripts/`. Statistics are calculated afterwards. These are saved in `stats.json` in the respective folder. Pure end to end latencies can be found in `latencies.csv`.

### Pick your desired parameter configuration

Afterwards, you can collect `stats.json` according to your desired parameter configuration:

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

### Analyze latency distribution + msg drops

If you want to analyze the empirical CDF/PDF of the latencies including message drops, `analyze_package_drops` is your friend. It is used in the same way as `collect_lat_csv.py`. Refer to `create_csvs_for_paper.sh` for use.

# Build Connext on Raspberry Pi

In general, if there are any build problems, refer to [here](https://github.com/ros2/rmw_connextdds/issues/10#issuecomment-800513412).

1. Ensure you have Ubuntu 18.04 installed on Raspi, **not Raspbian!**.
2. Download armxv8Linux4.4gcc5.4.0 version connext 6.0 from the RTI Download center. 
3. Follow the instructions of [this link](https://github.com/ros2/rmw_connextdds/issues/10#issuecomment-800513412).

**Ensure that the old rmw connext version is not installed!** In our case, the following error was thrown:

```console
Starting >>> connext_cmake_module
--- stderr: connext_cmake_module
CMake Error at cmake/Modules/FindConnext.cmake:257 (message):
  RTI code generator doesn't exist with path:
  '/opt/rti_connext_dds/bin/rtiddsgen'
Call Stack (most recent call first):
  CMakeLists.txt:9 (find_package)
```

# Steps to reproduce Paper Results

If you want to reproduce the paper results, proceed as follows:

0. Make sure you are in our docker container as described above.
1. `cd <ws> && . /opt/ros/foxy/setup.bash && . ws/install/setup.bash`
2. Set kernel parameters as described in the paper.
3. `cd <ws-root>/src/separate_processes/scripts/`
4. `bash run_all_benchmarks.sh`

This will take some time, roughly 60h.
Afterwards, post-process the results:

1. `python calc_e2e_lat.py --directory <parent-directory-of-results>`. **Attention**: Lots of RAM (>16GB) required.
2. Use the script `create_csvs_for_paper.sh`. The script should be self-explanatory.