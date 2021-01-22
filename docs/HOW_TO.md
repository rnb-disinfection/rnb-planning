# HOW TO

## Documents  
* The main instruction document is [ReadMe.md](../ReadMe.md)
* Other instruction documents are stored in docs/
    
## Doxygen
* To generate doxygen document, run following command (after installing doxygen following [docs/SUPPLEMENT_README.md](../docs/SUPPLEMENT_README.md)
    ```console
    cd "$RNB_PLANNING_DIR" && doxygen Doxyfile
    ```
* Doxygen documentation can be locally browsed from [doxygen/html/index.html](index.html)
* To host doxygen documents, run following command (port=5000 by default)
    ```console
    cd "$RNB_PLANNING_DIR"doxygen && python ./host_doxygen.py
    ```
