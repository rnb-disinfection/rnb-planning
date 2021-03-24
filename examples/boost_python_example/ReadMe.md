# Boost Python Example
This is a example for using boost-python to communicate between c++ and python in linux with cmake.  
The project includes basic template of boost-python communication, including basic wrapper of stl-container, which enables use of them in python.  

# Usage
* Build the project.
```bash
cmake -DCMAKE_BUILD_TYPE=Release . && make
```

* Test loading c++ library and using them in python
```bash
python test_loading.py
```