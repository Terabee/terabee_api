# TerabeeApi demo

Proof of concept of API for Terabee sensors. The idea is to simplify sensors usage on the client side by providing abstract layer between sensor communication protocol and client algorithms.

## To compile API

```
mkdir build
cd build
cmake ..
make
```

## To install API in the system

Compile api and then

```
sudo make install
```

## To uninstall API from the system

Use the `install_manifest.txt` file. For example:

```
sudo rm `cat install_manifest.txt`
```

## To use installed API in a separate project

```
find_package(TerabeeApi REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example ${TerabeeApi_LIBRARIES})
target_include_directories(example PUBLIC ${TerabeeApi_INCLUDE_DIRS})
```

## To generate doxygen documentation

Run the following command from inside the API folder:
```
doxygen
```

## To use in another ROS package (catkin worspace)

1. Add the API folder to the *src* folder of your workspace
2. In your *CMakelLists.txt*, add a find package statement:
```
find_package(
  catkin REQUIRED COMPONENTS
  TerabeeApi
)
```
3. In the same file, also add *TerabeeApi* as a catkin dependency:
```
catkin_package(DEPENDS TerabeeApi)
```
4. In your *package.xml* add TerabeeApi as a build and exec dependency. for instance:
```
<build_depend>roscpp</build_depend>
<build_depend>TerabeeApi</build_depend>
<exec_depend>TerabeeApi</exec_depend>
```
5. Follow the "To use installed API in a separate project" section for linking and including