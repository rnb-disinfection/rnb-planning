py\_etasl
=========

License
--------

LGPL v3, see LICENSE.txt

Introduction
------------

This package contains a python API to run eTaSL

Installation
------------

This ROS package requires the additional installation of cython:

    sudo apt install cython

Accessing in python
-------------------

-  Be sure that the devel/setup.bash of the appropriate workspace is
   sourced.
-  In python:

   ::

       import py_etasl_pkg.py_etasl as py_etasl
       e = py_etasl.etasl()
       ...

Quick guide to python virtual environments, pip and the jupyter notebooks
-------------------------------------------------------------------------

Installing
~~~~~~~~~~

The following installs a utility to manage python virtual environments:

    sudo apt install python-virtualenv sudo apt-get -y install
    python-pip python-dev

You can then create and activate a virtual environment:

    virtualenv ./venv source ./venv/bin/activate

Make sure that the python installation tool pip is up-to-date:

    sudo -H pip install --upgrade pip

And install numpy, scipy, matplotlib, and jupyter in a python virtual
environment:

    sudo -H pip install numpy scipy matplotlib

    sudo -H pip install jupyter

    sudo -H pip install cython

(the sudo before pip is not always necessary, it depends on your local
configuration).

Using
~~~~~

To activate the environment (each time you want to use jupyter):

    source ./venv/bin/activate

This configures the binary and python search path to prefer the python
configuration in the virtual environment. To deactivate the environment
type:

    deactivate

Moving the directory
~~~~~~~~~~~~~~~~~~~~

By default, the python virtual environment does not import the libraries
you have installed globally. This allows you to be completely
independent from the local system. You normally cannot move your python
virtual environment directory without any side-effects. It is best to
re-generate this directory in the new location. The following pip
commands can be useful:

Write your current installation to a requirements file:

    pip freeze > req.txt

Install everything specified in a requirements file:

    pip install -r req.txt

To start-up a python notebook, go to your source directory and type:

    jupyter notebook

This will start a jupyter notebook server and launch a browser-window
that accesses jupyter

Author
------

Erwin Aertbeliën
