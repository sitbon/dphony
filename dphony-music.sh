#!/bin/bash -e
./dphony.py -I 10.0.0.143 -i 239.255.0.82 -p 10082 -O 10.0.0.143 -o 239.255.0.81 -P 10081 --music $*
