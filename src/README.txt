Dependencies: Rosbridge
For kinetic:
sudo apt-get install ros-kinetic-rosbridge-suite

Running server
If you're running on a virtual machine make sure you have a bridged connection or portforwarded to 9090 (default server port) for use.
roslaunch rosbridge server rosbridge websocket.launch

You can run this on the same robot as well. I use python's simpleHTTP to run mine.

