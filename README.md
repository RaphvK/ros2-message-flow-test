# Trace complex N-to-M ROS 2 message flows with Eclipse Trace Compass

>This work is based on https://github.com/christophebedard/ros2-message-flow-analysis and was updated to ROS 2 jazzy. The purpose of this repository is to easily demonstrate the function of the corresponding Pull Request in [ros2_tracing](github.com/ros2/ros2_tracing).

1. Clone this repository recursively and check out the branch with modifications for message flow tracing:

    ```bash
    git clone --recursive https://github.com/RaphvK/ros2_tracing.git
    cd ros2_tracing && git checkout message-flow
    ```

2. Install required ROS dependencies and build the packages in a ROS workspace, e.g. in a Docker container:

    ```bash
    docker run --rm -it -v $(pwd):/workspace/src ros:jazzy
    ```

3. Execute the following command in the Docker container to setup the ROS workspace:

    ```bash
    # install dependencies
    apt update && rosdep update
    cd /workspace
    rosdep install -i --from-paths src -y

    # build and source the ROS packages
    colcon build
    source install/setup.bash
    ```

4. Run the nodes to produce trace data, which will be stored in th emounted folder `/workspace/src/traces`:

    ```bash
    ros2 launch test_publisher test_publisher_launch.py trace_path:=/workspace/src/traces
    # wait for a few mesages being received and sent before stopping the nodes with CTRL-C
    ```

5. On your host, install and start [Eclipse Trace Compass (with Incubator plugins)](https://eclipse.dev/tracecompass/)

    ```bash
    wget https://download.eclipse.org/tracecompass.incubator/stable-11.2/rcp/trace-compass-0.16.0-20251127-1956-linux.gtk.x86_64.tar.gz
    tar -xzf trace-compass-*-linux.gtk.x86_64.tar.gz
    rm trace-compass-*-linux.gtk.x86_64.tar.gz

    cd trace-compass
    ./tracecompass
    ```

6. Import the trace data and inspect the message flow:
  
  1. File --> Import ... --> Select root directory: **ros2-message-flow-test/traces** --> Check the trace folder in the list --> Finish
  2. Right-Click on "Traces" in the Project Explorer --> Open As Experiment --> ROS 2 Expermient (Incubator)
  3. Open Experiments --> trace-* --> Views --> ROS 2 Messages --> Right click on "Messages (incubator)" --> Open
  4. Inspect the message flow, hold CTRL and scroll to zoom in, hold Shift to scroll left/right. Click an one of the bars in the line of the "500 ms" timer, then click the "Follow this element" button above the graph to analyze the message flow.
  5. Open Experiments --> trace-* --> Views --> ROS 2 Message Flow --> Right click on "Message Flow (incubator)" --> Open
  6. You should see the message flow, where the red arrow from `/test_publisher/input_topic2` to `/test_publisher/output` is only visibile due to the [`TRACEPOINT()` in the source code](./test_publisher/src/test_publisher.cpp) that defines this dependency.

![Eclipse Trace Compass Screenshot](./screenshot.png)
