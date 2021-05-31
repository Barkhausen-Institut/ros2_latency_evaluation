#!/bin/bash

#############################
# change variables here
TEST=0  # set to 1, to choose the test param set.

if [ "1" -eq $TEST ]; then
    DDS_BACKENDS='rmw_fastrtps_cpp'
    INT_NODES_SET="100"
    F_PUBLISHER_SET="20"
    MSG_SIZE_SET='128b'
    QOS_RELIABILITY_SET='best-effort'

    TEST_DURATION=1
else
    DDS_BACKENDS='rmw_connextdds rmw_fastrtps_cpp rmw_cyclonedds_cpp'
    INT_NODES_SET="$(seq 1 21 2)"
    F_PUBLISHER_SET='1 10 20 30 40 50 60 70 80 90 100'
    MSG_SIZE_SET='128b 1kb 10kb 100kb 500kb'
    QOS_RELIABILITY_SET='best-effort reliable'

    TEST_DURATION=60
fi


####################################################
## change below only if you know what you are doing
####################################################
BASE="ros2 launch ../launch/chain.launch.py "

for QOS_RELIABILITY in $QOS_RELIABILITY_SET
do
    for MSG_SIZE in $MSG_SIZE_SET
    do
        for F_PUBLISHER in $F_PUBLISHER_SET
        do
            for BACKEND in $DDS_BACKENDS
            do
                for INT_NODES in $INT_NODES_SET
                do
                            timestamp=$(date +%Y-%m-%d_%H-%M-%S)
                            echo "[$timestamp]:"

                            command="INT_NODES=$INT_NODES \
                                PUB_FREQUENCY=$F_PUBLISHER \
                                MSG_SIZE=$MSG_SIZE \
                                QOS=$QOS_RELIABILITY \
                                DURATION=$TEST_DURATION \
                                RMW_IMPLEMENTATION=$BACKEND \
                                $BASE"
                    /bin/bash -c "$command"
                    echo "[$timestamp]: $command" >> log.txt
                done
            done
        done
    done
done
