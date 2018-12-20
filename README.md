Franka Panda
============
Redis driver for the Franka Panda.


Install
-------

This driver has been tested on Ubuntu 16.04 with C++14.

1. Set up your Panda robot and controller following the instructions on
   [this page](https://frankaemika.github.io/docs/getting_started.html).
   Connect your computer to the robot controller (not the robot directly).

3. Build the driver

   ```
   mkdir build
   cd build
   cmake ..
   make
   ```

Usage
-----

1. Open the robot interface by connecting to the ip address of the controller
   in your web browser (e.g. ```172.16.0.10```).

2. Open the User stop and the robot brakes through the web interface.

3. Launch a Redis server instance if one is not already running.

   ```
   redis-server
   ```

3. Open a terminal and go to the driver's ```bin``` folder.

   ```
   cd <path_to_franka-panda.git_directory>/bin
   ```

4. Launch the driver with a YAML configuration file.

   ```
   ./franka_redis_driver ../resources/default.yaml
   ```
