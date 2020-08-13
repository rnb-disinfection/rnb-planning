gRPC C++ - Building from source
===========================

# Clone the repository (including submodules)

```sh
 $ git clone --recursive -b v1.24.0 https://github.com/grpc/grpc
 $ cd grpc
 $ git submodule update --init --recursive
```

# Pre-requisites

## STEP2 (Linux)

Install Python 3.7. before install python please install openssl
```shell script
 $ sudo apt-get install openssl
```

Install managing packages
```shell script
 $ sudo apt-get install -y python-software-properties software-properties-common
 $ sudo apt-get install -y build-essential autoconf libtool pkg-config
 $ sudo apt-get install -y lsb
```
Then, fix python executor configuring in `/usr/bin/add-apt-repository` and `/usr/bin/lsb_release` header
```shell script
 #! /usr/bin/python3 --> /usr/bin/python3.4
```

Check packages
```shell script
 $ pkg-config --cflags protobuf grpc
```

Install clang-5.0 to STEP
```shell script
 $ wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | sudo apt-key add -
 $ sudo apt-add-repository "deb [arch=i686] http://apt.llvm.org/xenial/ llvm-toolchain-xenial-5.0 main"
 $ sudo apt-get update
 $ sudo apt-get install -y clang-5.0
 $ sudo apt-get install -y lld-5.0
```
* If error occurs on apt-get update, upload key
  * sudo apt-key adv keyserver --keyserver.ubuntu.com --recv-keys \<your key, on error message NO_PUBKY **XXXXX**\>

If you plan to build from source and run tests, install the following as well:
```shell script
 $ sudo apt-get install -y libgflags-dev libgtest-dev libc++-dev
```
Lastly, see the Protoc section below if you do not yet have the protoc compiler installed.

## Windows

To prepare for cmake + Microsoft Visual C++ compiler build
- Install Visual Studio 2015 or 2017 (Visual C++ compiler will be used).
- Install [Git](https://git-scm.com/).
- Install [CMake](https://cmake.org/download/).
- Install [Chocolatey](https://chocolatey.org/install).
- Install [Active State Perl](https://www.activestate.com/activeperl/) (`choco install activeperl`) - *required by boringssl*
- Install [Go](https://golang.org/dl/) (`choco install golang`) - *required by boringssl*
- Install [yasm](http://yasm.tortall.net/) and add it to `PATH` (`choco install yasm`) - *required by boringssl*
- (Optional) Install [Ninja](https://ninja-build.org/) (`choco install ninja`)

## Protoc

By default gRPC uses [protocol buffers](https://github.com/google/protobuf),
you will need the `protoc` compiler to generate stub server and client code.

If you compile gRPC from source, as described below, the Makefile will
automatically try compiling the `protoc` in third_party if you cloned the
repository recursively and it detects that you do not already have 'protoc' compiler
installed.

If 'protoc' compiler has not been installed, following commands can be used for installation.

```shell script
$ cd grpc/third_party/protobuf
$ ./autogen.sh        # generated the configure script
$ ./configure or ./configure CXXFLAGS="-pthread"
$ make
$ make check
$ sudo make install   # 'make' should have been run by core grpc
$ sudo ldconfig       # refresh shared library cache.
```

For more information of installing protobuf, please refer to its [github](https://github.com/protocolbuffers/protobuf/blob/master/src/README.md)

# Build from source

In the C++ world, there's no "standard" build system that would work for in all supported use cases and on all supported platforms.
Therefore, gRPC supports several major build systems, which should satisfy most users.

Note that this section only covers the build of gRPC itself, not the installation. See the [How to use](https://github.com/grpc/grpc/tree/master/src/cpp#to-start-using-grpc-c) instructions
for guidance on how to add gRPC as a dependency to a C++ application (there are several ways and system wide installation is often not the best choice).

## make: STEP2 (Linux)

Fix error :
https://stackoverflow.com/questions/53586540/c-terminate-called-after-throwing-an-instance-of-stdsystem-error


From the grpc repository root
```shell script
 $ make
```
NOTE: if you get an error on linux such as 'aclocal-1.15: command not found', which can happen if you ran 'make' before installing the pre-reqs, try the following:
```shell script
$ git clean -f -d -x && git submodule foreach --recursive git clean -f -d -x
$ [sudo] apt-get install build-essential autoconf libtool pkg-config
$ make
```

## cmake: Windows, Using Visual Studio 2015 or 2017 (can only build with OPENSSL_NO_ASM).
When using the "Visual Studio" generator,
cmake will generate a solution (`grpc.sln`) that contains a VS project for 
every target defined in `CMakeLists.txt` (+ few extra convenience projects
added automatically by cmake). After opening the solution with Visual Studio 
you will be able to browse and build the code.
```
> @rem Run from grpc directory after cloning the repo with --recursive or updating submodules.
> md .build
> cd .build
> cmake .. -G "Visual Studio 14 2015"
> cmake --build . --config Release
```

## cmake: Windows, Using Ninja (faster build, supports boringssl's assembly optimizations).
Please note that when using Ninja, you will still need Visual C++ (part of Visual Studio)
installed to be able to compile the C/C++ sources.
```
> @rem Run from grpc directory after cloning the repo with --recursive or updating submodules.
> md .build
> cd .build
> call "%VS140COMNTOOLS%..\..\VC\vcvarsall.bat" x64
> cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
> cmake --build .
```
In my case, run command as (with installed cygwin-win62 and VS2019 Community):
```
VS160COMNTOOLS
> & 'D:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvarsall.bat' x64
or
> call "%VS160COMNTOOLS%..\..\VC\vcvarsall.bat" x64
> & "C:\cmake-win64\bin\cmake.exe" .. -G"Visual Studio 16 2019" -DCMAKE_BUILD_TYPE=Release
> & "C:\cmake-win64\bin\cmake.exe" --build .
```
