# twist_streamer
ROS2 node to stream cmd_vel values taken from a CSV file

The CSV file should be composed of 3 columns: time, linear velocity, and angular velocity.\
Time values do not need to be equally spaced, and generated using a period multiple of 1 millisecond.\
The CSV file should be in the data folder, and the name should be set in the launch file as a paramenter of the node.

An example of CSV file and of Matlab script to generate a CSV file is provided in the data folder.

In order to interpolate and resample data imported from CSV file, this package uses linear splines made available from ALGLIB. In order to run the package you must first install ALGLIB following the instructions available at [ALGLIB](https://www.alglib.net/) website or at the [alglib-cmake](https://github.com/ROSETEA-lab/alglib-cmake) repository.
