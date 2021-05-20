#!/bin/bash
PKG_DROPS_ANALYSIS=1

DATA_DIRECTORY=$1
RES_DIRECTORY=$2

DDS_BACKENDS='fastrtps cyclonedds connext'
F_PUBLISHER_SET='100'
MSG_SIZE_SET='128b 500kb'
QOS_RELIABILITY_SET='best-effort'

if [ "1" -eq $PKG_DROPS_ANALYSIS ]; then
    BASE_PYTHON_COMMAND='python analyze_package_drops.py'
    ADDITIONAL_ARGS='--pkg-errors True --cdf True --pdf True'
else
    BASE_PYTHON_COMMAND='python collect_lat_csv.py'
    ADDITIONAL_ARGS=''
fi

for DDS_BACKEND in $DDS_BACKENDS
do
    for QOS_RELIABILITY in $QOS_RELIABILITY_SET
    do
        for F_PUBLISHER in $F_PUBLISHER_SET
        do
            for MSG_SIZE in $MSG_SIZE_SET
            do
                command="$BASE_PYTHON_COMMAND --directory $DATA_DIRECTORY \
                                              --res-dir $RES_DIRECTORY \
                                              --rmw $DDS_BACKEND \
                                              --reliability $QOS_RELIABILITY \
                                              --msg-size $MSG_SIZE \
                                              --f $F_PUBLISHER \
                                              $ADDITIONAL_ARGS"
                /bin/bash -c "$command"
            done
        done
    done
done
