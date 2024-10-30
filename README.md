# ASDP_Core_Module

This repository implements the **Core Module** of the ASDP project, controlling a camera system and
producing data for one or more network-connected clients.

The **TR-006v16_Software_Architecture** document describes the overall architecture of the project
and includes diagrams showing how the modules interact. It also includes descriptions of the APIs
used to communicate between modules.

## System setup

This module derives from the **CoreServerBase** class, which is defined in the **ASDP_Core_API** library.
That library must have been installed on the system before this module can be compiled.  The base class
implements most of the functionality of the server, so this module only needs to override virtual methods,
update the server state, and handle operations that continually run (sending temperature and pose messages).
The library is available at https://github.com/arizonaCameraLab/ASDP_Core_API 

```
sudo apt install cmake
cd; mkdir src; cd src; git clone https://github.com/arizonaCameraLab/ASDP_Core_API
cd; mkdir -p build/ASDP_Core_API; cd build/ASDP_Core_API
cmake ../../src/ASDP_Core_API
make
sudo make install
```

This module uses an instance of the **ASDP_BSP_API** to control the hardware, so an instance of that
must have been installed on the system before this module can be compiled.
A simulated library is available at https://github.com/arizonaCameraLab/ASDP_Camera_Simulator

One way to do this is to install the Camera_Simulator repository, as follows:

```
sudo apt install cmake
cd; mkdir src; cd src; git clone --recursive https://github.com/arizonaCameraLab/ASDP_Camera_Simulator
cd; mkdir -p build/ASDP_Camera_Simulator; cd build/ASDP_Camera_Simulator
cmake ../../src/ASDP_Camera_Simulator
make
sudo make install
```

Another way is to install the actual BSP library for the camera being used, once these become available.

## Building

Once the above installations are complete, CMake is used to configure the build.
The following commands can be used to build the project:

```
sudo apt install cmake
cd; mkdir src; cd src; git clone https://github.com/arizonaCameraLab/ASDP_Core_Module
cd; mkdir -p build/ASDP_Core_Module; cd build/ASDP_Core_Module
cmake ../../src/ASDP_Core_Module
make
```

## Installing

The application can be installed on the system by running the following command:
```
sudo make install
```

The dynamically-loaded library for ASDP_BSP must be copied into the same directory as the executable or into
a location specified in the load-library path.

## Running


The *ASDP_Core_Module* program runs the server and it must be given a command-line argument specifying
the network name or IP address of the network interface card to use (for example, *localhost* for testing).
