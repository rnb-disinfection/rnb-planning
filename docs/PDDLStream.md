# PDDL Stream Install Process

Original version link
[link](https://github.com/caelan/pddlstream.git)

## Install PDDL STream
```bash
cd ~/Projects \
&& git clone --recursive https://github.com/caelan/pddlstream.git \
&& cd pddlstream \
&& git submodule update --init --recursive \
&& ./FastDownward/build.py release64
```

## Install Pybullet
```bash
pip install pybullet numpy scipy \
&& pip3 install pybullet numpy scipy
```

## Examples
* PR2 TAMP: `pddlstream$ python -m examples.pybullet.tamp.run`
* PR2 Cleaning and Cooking: `pddlstream$ python -m examples.pybullet.pr2.run`
* Turtlebot Rovers: `pddlstream$ python -m examples.pybullet.turtlebot_rovers.run`
* PR2 Rovers: `pddlstream$ python -m examples.pybullet.pr2_rovers.run`
* PR2 Planning and Execution: `pddlstream$ python -m examples.pybullet.pr2_belief.run`
* Kuka Cleaning and Cooking: `pddlstream$ python -m examples.pybullet.kuka.run`

#### lru_cache error
* **functools.lru_cache** is added in **Python3.3**, and usage has been changed in **Python3.8**
* To use with **Python<=3.7**,
    * Go to the line where error has raised.
    * Add parenthesis after ***@cache***,  so that the line becomes ***@cache()***
* To use with **Python2.7**,
    * Go to the line where error has raised.
    * Change ***functools*** to ***functools32***
    * Add parenthesis after ***@cache***,  so that the line becomes ***@cache()***