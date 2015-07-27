#!/usr/bin/env bash

PYAIML_DIR=$HOME/catkin_ws/src/ai/sandbox/chatbot_ros_node/pyaiml

# TODO Figure out where the latest repository for pyaiml is.
# Until then, copy the distribution from another computer, then run this script to install it.
#git clone git://pyaiml.git.sourceforge.net/gitroot/pyaiml/pyaiml $PYAIML_DIR
#git clone https://github.com/creatorrr/pyAIML.git $PYAIML_DIR
CUR_DIR=`pwd`
cd $PYAIML_DIR
sudo python setup.py install
cd $CUR_DIR
