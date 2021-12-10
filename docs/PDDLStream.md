# PDDL Stream Install Process

Original version link
[link](https://github.com/caelan/pddlstream.git)

## Install PDDL STream
```bash
cd ~/Projects \
&& git clone --recursive https://github.com/caelan/pddlstream.git \
&& cd pddlstream \
&& git checkout main \
&& git submodule update --init --recursive \
&& ./downward/build.py
```

## Add path to environment 
```bash
export PDDL_STREAM_DIR=$HOME/Projects/pddlstream/ \
&& echo 'export PDDL_STREAM_DIR=$HOME/Projects/pddlstream/' >> ~/.bashrc
```

## Install Pybullet
```bash
pip install pybullet numpy scipy \
&& pip3 install pybullet numpy scipy
```

## Edit to strictly obey timeout
* Go to *pddlstream/algorithms/skeleton.py* and find STANDBY = None and set STANDBY = False
* This makes empty stream removed from stream queue. Possibly remove chance for the stream to be used by later-discovered skeletons
```python
STANDBY = False
```
* Go to *pddlstream/algorithms/focused.py*
* Find the line that calls the function ***iterative_plan_streams*** (around line 177)
* Add a new argument *time_remain=max_time-store.elapsed_time()*
* Go to definition of ***iterative_plan_streams*** in pddlstream/algorithms/refinement.py
* Add a new argument *time_remain=None*
* Change the condition for failure return from
```python
if final_depth == 0:
```
to
```python
if final_depth == 0 or elapsed_time(start_time)>time_remain:
```

## To get plan summary
* Go to *pddlstream/algorithms/common.py*
* Find the function ***export_summary(self)***
* Change "return = {" to "SolutionStore.last_log = {"
* Add "return SolutionStore.last_log" at the end of the function
* The function should look like this:
```python
def export_summary(self):
    SolutionStore.last_log = {
    ...
    }
    return SolutionStore.last_log
```
* Get the log from pddlstream.algorithms.common.SolutionStore.last_log as follows:
```python
from pddlstream.algorithms.common import SolutionStore
print(SolutionStore.last_log)
```


## Integrated Usage
* Check 4.3 TaskPlanner-PDDLStream section in [release/4.PlanningPipeline.ipynb](../release/4.PlanningPipeline.ipynb) for detailed usage.


## Examples
* PR2 TAMP: `python3 -m examples.pybullet.tamp.run -viewer`
* PR2 Cleaning and Cooking: `python3 -m examples.pybullet.pr2.run -viewer`
* Turtlebot Rovers: `python3 -m examples.pybullet.turtlebot_rovers.run -viewer`
* PR2 Rovers: `python3 -m examples.pybullet.pr2_rovers.run -viewer`
* PR2 Planning and Execution: `python3 -m examples.pybullet.pr2_belief.run -viewer`
* Kuka Cleaning and Cooking: `python3 -m examples.pybullet.kuka.run -viewer`

#### lru_cache error
* **functools.lru_cache** is added in **Python3.3**, and usage has been changed in **Python3.8**
* To use with **Python<=3.7**,
    * Go to the line where error has raised.
    * Add parenthesis after ***@cache***,  so that the line becomes ***@cache()***
* To use with **Python2.7**,
    * Go to the line where error has raised.
    * Change ***functools*** to ***functools32***
    * Add parenthesis after ***@cache***,  so that the line becomes ***@cache()***