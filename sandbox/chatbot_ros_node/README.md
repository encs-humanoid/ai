ENCS Humanoid / A.I. Chatbot Test
======



### Running the Test

This document assumes you have pulled the A.I. code from the Github repo to your machine.

On your computer, assuming you are running just one ROS machine, open a terminal and run:

```sh
roscore
```

You should see messages emitted, including "started core service".

Open another terminal, cd to the sandbox/chatbot_ros_node directory and run:

```sh
./ai_respond_node.py
```

This starts up the ROS node that acts as the robot's A.I.

Finally, open one more terminal, cd to the sandbox/chatbot_ros_node directory, and run:

```sh
./human_input.py
```

In this last terminal, you should now be able to type a sentence, hitting the return key to send it to the A.I. node. You will see the response in the second terminal you opened. In the physical robot, this human input node would be replaced by a node that captures spoken words. Additionally, another node would be added to speak the response out loud.
