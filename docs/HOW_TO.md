# HOW TO

## Documents  
* The main instruction document is [ReadMe.md](../ReadMe.md)
* Other instruction documents are stored in docs/
* Lecture materials are uploaded on docs/Lectures (2022.01.25)
  
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
* Now you can browse doxygen documents from other computers in the local network at \<IP\>:\<PORT\>

## Scripts
* Scripts to use or test the framework are in src/scripts and src/code_test
* Try src/scripts/\[TUTORIAL\] ConstrainedTaskPanning(Sweep).ipynb to learn basic usage

## NOTE for Developers
* Please follow PEP8 coding convention.
* Please add doxygen-style comments to any function and class that you implement.
* Please follow git-flow branching strategy.
    * master: latest release
    * develop: main development branch
    * feature: to add specific feature
    * release: to prepare release
    * hotfix: quick bugfix from a release version