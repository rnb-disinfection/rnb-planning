Indy Framework v2.3.0-alpha
=========================

This is the IndyFramework 2.3.0 (alpha) project for Indy robots developed by Neuromeka's Software Team.
This proejct is managed/built by CMake.

## Requirements
### New toolchain (6.5) and libraries for Host PC
You can download from `\\192.168.0.83\Season5\0_Share 공통\80_info\Software\NEW_SDK_FRAMEWORK_3_0`
Please use latest version.
#### Windows Host
* SetupNRMKPlatformPC2_v3.0.2_b20190627.exe
* SetupNRMKIndySDK_v3.0.5_20190627.exe
    
#### Ubuntu host
* NRMKPlatformPC2_v3.0.2_b20190627.tar.gz
* NRMKIndySDK_v3.0.5_b20190627.tar.gz

### New toolchain (6.5) and libraries for Target (STEP2)
* STEP2-gcc-6.5-package_v1.0.1.deb
* STEP2-3rdparty-libs-package_v1.0.1.deb
* fix_stdlibc++.sh

## Project Setting Options
### Quick Setting
#### Example
* Execute cmake command line with below options:
~~~~
-DROBOT_NAME=Indy7 -DSIMULATION=ON
~~~~
#### Options
| Option | Description | Values | Default |
|:---|:---|:---:|:---:|
| `ROBOT_NAME` | Build Target Robot | `Indy7` `IndyRP2` | `Indy7` |
| `SIMULATION` | Build as Simulation Mode  | `ON` `OFF` | `OFF` |
| `LOGGER_CONSOLE` | Use Console Logging | `ON` `OFF` | `ON` |
| `IMPEDANCE` | Use Impedance Control (Beta) | `ON` `OFF` | `OFF` |
| `EMODI` | Use eModi (Beta) | `ON` `OFF` | `OFF` |
| `DEPLOYMENT_SHOW_VERSION` | Include Version to each Binary and Library | `ON` `OFF` | `OFF` |

### Detail Setting
* Modify options in CMakeList.txt at root directory:
~~~~
set(ROBOT_NAME "Indy7" CACHE STRING "Supported Robot: Indy7 | IndyRP2")
set_property(CACHE ROBOT_NAME PROPERTY STRINGS "Indy7" "IndyRP2")

option(SIMULATION "Setting Robot Interface" OFF)
option(LOGGER_CONSOLE "Enable Console logging for LoggerTask" ON)

option(IMPEDANCE "Enable Impedance Control" OFF)
option(EMODI "Use eModi" OFF)

CMAKE_DEPENDENT_OPTION(USE_IO_BOARD "Use IO Board" ON "NOT SIMULATION" OFF)
CMAKE_DEPENDENT_OPTION(USE_EMG_IO "Use EMG signal" ON "NOT SIMULATION" OFF)
CMAKE_DEPENDENT_OPTION(USE_STEP_GPIO "Use STEP GPIO Signal only for Simulation" OFF "NOT SIMULATION" ON)
CMAKE_DEPENDENT_OPTION(USE_DC_MODE "Ecat DC Mode" OFF "NOT SIMULATION" OFF)
~~~~
* Be CAREFUL modifying options!

## Project Deployment (automatically generated)
### Command
~~~~
make install
~~~~

### Deployment Folder
* Local
~~~~
set(DEPLOYMENT_DIR "deployment")

set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/${DEPLOYMENT_DIR}")
~~~~
* Target
~~~~
set(DEPLOYMENT_TARGET_DIR "/home/user/release/TasksDeployment")
~~~~

### Deployment Files
* indyDeploy.json
* TaskManager
* TaskUpdator
* RemoteMT
* [__Robot__]ControlTask.so or [__Robot__]ControlTask_sim.so 
* [__Robot__]CadkitTask.so
* [__DOF__]LoggingTask.so
* [__DOF__]ModbusTCPTask.so
