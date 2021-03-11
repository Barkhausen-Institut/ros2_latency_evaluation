# PingPongComparison

https://redmine.adbi.barkhauseninstitut.org/issues/1405

## Start multiprocesses:
From the directory `src/separate_processes`

```console
$ <parameters> ros2 launch launch/chain.launch.py
```

##  Overview of parameters

- `INT_NODES=1`: Integer number of nodes to spawn between `start_node` and `end_node`
- `PUB_FREQUENCY=1`: Floating point number of frequency with which the publisher should send its message, in Hz.
- `MSG_SIZE=100b`: Msg size of the topic. Supported values are
  - 100b
  - 1kb
  - 10kb
  - 100kb
  - 500kb