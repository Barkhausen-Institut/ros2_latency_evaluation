# PingPongComparison

https://redmine.adbi.barkhauseninstitut.org/issues/1405

## Start multiprocesses:
From the directory `src/separate_processes`

```console
$ INT_NODES=10 ros2 launch launch/chain.launch.py
```

Here, `INT_NODES` describes the number of intermediate nodes.

### Change Publisher Freuqency

Add to the above command:

```console
$ INT_NODES=10 PUB_FREQUENCY=10 ros2 launch launch/chain.launch.py
```

`PUB_FREQUENCY` denotes the frequency with which to publish messages. It must be > 1.