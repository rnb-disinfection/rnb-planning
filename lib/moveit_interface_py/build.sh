rm -rf Release
mkdir Release && cd Release
cmake -DCMAKE_BUILD_TYPE=Release .. && make
cd .. && cp ./Release/moveit_interface_py.so ./ ## && rm -rf Release
