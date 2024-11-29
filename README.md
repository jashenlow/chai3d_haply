# README

This repository is a fork of the original CHAI3D, which includes additional support for the Haply Inverse3 haptic device.  
Implementations are also based on Haply's CHAI3D demos.  
The following library versions are used in this repository:  
- CHAI3D: **3.3.0**  
- Haply.HardwareAPI: **0.1.11**  

## Building CHAI3D
1. Configure with CMake.  
    ```
    cmake -S . -B "build"

    # OR

    # Configure building with examples (Default is OFF)
    cmake -DBUILD_EXAMPLES=ON -S . -B "build"
    ```
2. Build library  
    ```
    # For Debug
    cmake --build "build" --config Debug

    # OR

    # For Release
    cmake --build "build" --config Release
    ```
    Optional if building in Windows:
    ```
    cd build

    # Open CHAI3D.sln in Visual Studio, then build individual components from there.
    ```


## Preprocessor Definitions
The following preprocessor definitions have been added to provide more customization options:  
1. **HAPLY_HANDLE_HZ**  
    - Modifies the handle's thread update rate in Hz. (Should typically be 50Hz to 60Hz)  

For them to take effect, ensure that they are defined before including chai3d.h in your source file.  
You can also refer to [CHaplyDevices.h](src/devices/CHaplyDevices.h).  
```
#define HAPLY_HANDLE_HZ 50
#include "chai3d.h"
```

## Additional Files
* The Haply HardwareAPI library files can be found in [externals/haply](externals/haply).  
* Implementation files can be found in [CHaplyDevices.h](src/devices/CHaplyDevices.h), and [CHaplyDevices.cpp](src/devices/CHaplyDevices.cpp).  

## Links
1. [Original CHAI3D](https://github.com/chai3d/chai3d)  
2. [Haply CHAI3D Demos](https://gitlab.com/Haply/public/chai3d-demos)  
3. [Haply HardwareAPI Releases](https://develop.haply.co/releases/cpp)  
4. [Haply HardwareAPI Reference](https://docs.haply.co/hardwareAPI/CPP/)  
5. [Haply Inverse3 Haptic Device](https://www.haply.co/inverse3)  