Franka Panda
============
Redis driver for the Franka Panda.


Install
-------

This driver has been tested on Ubuntu 16.04 with C++14.

1. Set up your Panda robot and controller following the instructions on
   [this page](https://frankaemika.github.io/docs/getting_started.html).
   Connect your computer to the robot controller (not the robot directly).

2. CMake 3.6 or higher is required to download the external dependencies. Ubuntu
   16.04 comes with CMake 3.5. The easiest way to upgrade is through pip:

   ```
   pip install cmake
   ```

3. Build the driver

   ```
   cd <franka-panda.git>
   mkdir build
   cd build
   cmake ..
   make
   ```

Usage
-----

1. Open the robot interface by connecting to the ip address of the controller in
   your web browser (e.g. ```172.16.0.2```).

2. Open the User stop and the robot brakes through the web interface.

3. Launch a Redis server instance if one is not already running.

   ```
   redis-server
   ```

3. Open a terminal and go to the driver's ```bin``` folder.

   ```
   cd <franka-panda.git>/bin
   ```

4. Launch the driver with a YAML configuration file.

   ```
   ./franka_redis_driver ../resources/default.yaml
   ```

Franka Panda Dynamics Library
-----------------------------

In addition to the Redis driver, this repo provides C++ and Python bindings to
the internal Franka Panda dynamics binary.

### C++
1. If the driver has already been compiled, you can use the following lines in
   your CMakeLists.txt:

   ```
   find_package(franka_panda REQUIRED)
   target_link_libraries(<target> PRIVATE franka_panda::franka_panda)
   ```

2. Include the following header in your source code:

   ```
   #include <franka_panda/franka_panda.h>
   ```

### Python
1. Activate your Python virtual environment (e.g. `pipenv`).

2. Locally install the `frankapanda` module:

   ```
   cd <franka-panda.git>
   pip install -e .
   ```

3. Import the `frankapanda` module in your Python code:

   ```
   import frankapanda
   ```

Examples
--------

To run the example control apps, you will need to perform the following
additional steps.

1. Download and compile `spatial_dyn`.
2. Rebuild the driver (step 3 in the **Install** section above).

The apps are provided in both C++ and Python:

### C++

1. Build the example app:

   ```
   cd <franka-panda.git>/examples/opspace
   mkdir build
   cd build
   cmake ..
   make
   ```

2. Run the app:

   ```
   cd <franka-panda.git>/examples/opspace/bin
   ./franka_panda_opspace ../../../resources/franka_panda.urdf --sim
   ```

### Python

1. Activate the Python virtual environment above where you installed the
   `frankapanda` module.

2. Locally install the `spatialdyn` module (look in
   `~/.cmake/packages/spatial_dyn` for a hint of where it's located):

   ```
   cd <spatial-dyn.git>
   pip install -e .
   ```

3. Run the app:

   ```
   cd <franka-panda.git>/examples/opspace/python
   ./franka_panda_opspace.py ../../../resources/franka_panda.urdf --sim
   ```
