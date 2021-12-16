## Setup Process
0. Get archived files from https://drive.google.com/drive/folders/1y_5ndU2nOotaXbiVuKiQysHnB_wZt7pk?usp=sharing

1. Install protobuf, grpcio
```bash
python3 -m pip install protobuf-3.19.1-cp36-cp36m-manylinux_2_17_x86_64.manylinux2014_x86_64.whl
python3 -m pip install grpcio-1.42.0-cp36-cp36m-manylinux_2_17_x86_64.manylinux2014_x86_64.whl
python3 -m pip install grpcio_tools-1.42.0-cp36-cp36m-manylinux_2_17_x86_64.manylinux2014_x86_64.whl
```

2. Compile protobuf
```bash
cd src/scripts/demo_202107/demo_proto
protoc -I=. --python_out=. DisinfectionOperation.proto
python3 -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. DisinfectionOperation.proto
```
