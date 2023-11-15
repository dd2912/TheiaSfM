#!/bin/sh
( cd /home/fabien/Documents/TheiaSfM/build ; make -j20 ; make install )
rm -rf /home/fabien/Documents/TheiaSfM/data_theia/output
/home/fabien/Documents/TheiaSfM/build/bin/build_reconstruction --flagfile=./flags.txt


