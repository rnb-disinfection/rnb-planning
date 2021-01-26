# HOW TO

## Documents  
* The main instruction document is [ReadMe.md](../ReadMe.md)
* Other instruction documents are stored in docs/
  
## Doxygen
* To generate doxygen document, run following command (after installing doxygen from [docs/SUPPLEMENT_README.md](../docs/SUPPLEMENT_README.md)
  ```bash
  cd "$RNB_PLANNING_DIR" && doxygen Doxyfile
  ```
* Doxygen documentation can be locally browsed from [doxygen/html/index.html](index.html)
* To host doxygen documents, run following command (port=5000 by default)
  ```bash
  cd "$RNB_PLANNING_DIR"doxygen && python ./host_doxygen.py
  ```
* Now you can browse doxygen documents from other computers in the local network at \<IP\>:\<PORT\>/index.html

## Scripts
* Scripts to use or test the framework are in src/scripts
