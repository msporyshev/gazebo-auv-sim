gazebo-auv-env
==============

Simulation tools for Far Eastern Federal University autonomous underwater vehicle.


###Dependencies:
+ CMake >= 2.8
+ Boost >= 1.49
+ pkg-config
+ gazebo >= 1.9
+ protobuf >= 2.5.0
 
###Building:
To use with Carnegie Mellon IPC messages adapter:
```bash
export IPC_MSG_INCLUDE_DIR=<dirname> 
```

To build with make:
```bash
mkdir build
cd build
cmake ../ 
make
``` 

###Usage:
```bash
cd ./gazebo-plugins
gazebo robosub_auv.sdf
```

To use with FEFU AUV ipc messages you need to run adapter:
```bash
./ipc-gazebo-adapter/adapter
```

To view adapter options:
```bash
./ipc-gazebo-adapter/adapter --help
```
