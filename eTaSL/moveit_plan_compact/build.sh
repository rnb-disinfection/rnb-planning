rm -rf Release
mkdir Release && cd Release
cmake -DCMAKE_BUILD_TYPE=Release .. && make
cd .. && cp ./Release/moveit_plan_compact.so ./ && rm -rf Release
