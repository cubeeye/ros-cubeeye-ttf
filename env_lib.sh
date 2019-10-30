#!/bin/bash

# meerecompany driver install.
# product VID, PID setting
# TTF_API 0.0.1ver

CURDIR=`pwd`

export TTF_SDK_PATH=$CURDIR
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CURDIR/lib
