#!/usr/bin/env bash

PYAIML_DIR=/vagrant/sandbox/chatbot_ros_node/pyaiml

git clone git://pyaiml.git.sourceforge.net/gitroot/pyaiml/pyaiml $PYAIML_DIR
CUR_DIR=`pwd`
cd $PYAIML_DIR
sudo python setup.py install
cd $CUR_DIR
