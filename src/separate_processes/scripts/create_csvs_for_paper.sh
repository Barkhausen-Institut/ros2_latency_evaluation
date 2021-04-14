#!/bin/bash

DATA_DIRECTORY=$1
RES_DIRECTORY=$2

DDS_BACKENDS='cyclone'
F_PUBLISHER_SET='1 10 40 60 80 100'
MSG_SIZE_SET='128b 1kb 10kb 100kb 500kb'
QOS_RELIABILITY_SET='best-effort'

BASE_PYTHON_COMMAND='python collect_lat_csv.py'

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
                                              --f $F_PUBLISHER"
                /bin/bash -c "$command"
            done
        done
    done
done