# eTaSL  
* Follow install process below (reference: https://etasl.pages.gitlab.kuleuven.be/install-new.html)  
    ```
    cd ~ \
    && git clone --recursive https://gitlab.kuleuven.be/rob-expressiongraphs/docker/etasl-install.git etasl \
    && cd etasl \
    && source install-dependencies.sh
    ```
* **ADD** "source $HOME/orocos-install/orocos-2.9_ws/install_isolated/setup.bash" on top of ~/.bashrc  
* switch gcc and g++ version to 7 before installing etasl
    ```
    sudo update-alternatives --config gcc && sudo update-alternatives --config g++  
    ```
* install etasl base
    ```
    source $HOME/etasl/etasl-install.sh
    ```
* switch gcc and g++ version to 5 before installing etasl-py
    ```
    sudo update-alternatives --config gcc && sudo update-alternatives --config g++  
    ```
* install etasl-py base
    ```
    source $HOME/etasl/ws/etasl/devel/setup.sh \
    && source python-etasl-install.sh \
    && source $HOME/etasl/ws/etasl-py/devel/setup.bash \
    && echo 'source $HOME/etasl/ws/etasl-py/devel/setup.bash' >> ~/.bashrc \
    ```
* switch gcc and g++ version back to 7 before installing etasl-py
    ```
    sudo update-alternatives --config gcc && sudo update-alternatives --config g++  
    ```
* ***If eTaSL is slow***, re-compile packages in release mode (has to be under 300ms with 200 constraints 500 step)  