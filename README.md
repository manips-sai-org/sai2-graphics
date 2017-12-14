# sai2-graphics

This is SAI2 graphics library for simulation displays.
It uses [CHAI3D](https://github.com/manips-sai-org/chai3d)

## Build instructions 
You can use the provided install script for automatic install
```
sh install.sh
```
Or you can install manually :
 * First, you need to initialize the chai3d submodule and then make it
```
git submodule update --init
cd chai3d
mkdir build
cmake .. && make -j8
cd ../..
```
 * Then you can make sai2-graphics from base directory
 ```
 mkdir build
 cd build
 cmake .. && make -j8
 cd ..
 ```

 ## run the examples
Go to the build/examples/one_of_the_examples folder and run the example. For example 1 :
```
cd build/examples/01-parse_world_and_robot
./01-parse_world_and_robot
```