#! /usr/bin/env python
#
# Simple python script that demonstrates using the lr4ranger C library for
# communicating with the Porcupine LR4 Laser Rangerfinder.
#
# This script shows how to load the library dynamically and make appropriate
# calls into it.  It opens a connection to the ranger, starts collecting
# data into a file, sleeps for awhile, stops collecting and disconnects.
#
import ctypes
import time
import sys
import os

#
# Full path to C library (we assume it's in ../src directory here)
#
library_path = os.path.abspath(os.path.dirname(__file__) +
    '/../src/liblr4ranger.so.1')

#
# load the lr4ranger dynamic library.
#
lr4ranger_lib = ctypes.CDLL(library_path)

#
# try to open USB connection to ranger
#
result = lr4ranger_lib.lr4ranger_open()
if result != 0:
    print 'Failed to open ranger: ' + str(result)
    sys.exit(1)

#
# start ranger collecting data to a file
#
data_file = '/tmp/ranger.dat'
result = lr4ranger_lib.lr4ranger_start_collecting(data_file)
if result != 0:
    print 'Failed to start collecting: ' + str(result)

#
# You could do other stuff here, e.g. move servos
#
print 'collecting...'
time.sleep(5)
print 'done collecting'

#
# We're done moving servos, stop collecting data
#
result = lr4ranger_lib.lr4ranger_stop_collecting()
if result != 0:
    print 'Failed to stop collecting: ' + str(result)

#
# Always close the USB connection!
#
result = lr4ranger_lib.lr4ranger_close()
if result != 0:
    print 'Failed to stop collecting: ' + str(result)

