#!/bin/sh

. /tmp/build/devel/setup.sh
exec roslaunch drone_master_messenger messenger.launch 
