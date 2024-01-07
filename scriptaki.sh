#!/bin/bash

# Stop the script if any command fails
set -e

# Step 0: Compile the module
make

# Step 1a: Remove previous module
# rmmod ./lunix.ko

# Step 1b: Insert the module
insmod ./lunix.ko

# Step 2: Create the device files
./mk-lunix-devs.sh

# Step 3: Attach the Lunix TNG to the serial port
./lunix-attach /dev/ttyS1

