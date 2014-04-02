gazebo-auv-sim
==============

Simulation tools for Far Eastern Federal University autonomous underwater vehicle.


###Dependencies:
+ CMake >= 2.8
+ Boost >= 1.49
+ pkg-config
+ gazebo >= 1.9
+ protobuf >= 2.5.0
 
###Building:
To use with Carnegie Mellon IPC messages:
```bash
export IPC_MSG_INCLUDE_DIR=<directory_with_ipc_messages> 
```

To build with make:
```bash
mkdir build
cd build
cmake ../ 
make install
``` 

###Usage:
In build directory:
```bash
cd ./launch
gazebo robosub_auv.sdf
```

To use with FEFU AUV ipc messages you need to run adapter:
```bash
./launch/bin/adapter
```

To view adapter options:
```bash
./launch/bin/adapter --help
```
