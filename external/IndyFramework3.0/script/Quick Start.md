# 1. Quick Start

## Prerequisites

- IndyFramework 3.0 개발환경 (STEP)
    - Package: `protobuf`, `grpc`
    - Python 3.7.x : `protoc`, `grpcio`, `grpcio-tools`
- 자세한 설치방법은 [설치 가이드](../install/Install.md) 참조


## The First Script

아래 코드(`quick_start.py`)는 IndyScript를 처음 사용하는 사용자들을 위한 간단한 예제 프로그램입니다. 이 프로그램을 실행하면 로봇은 홈 위치(Home Position)로 이동 후 x축으로 0.2 미터만큼 직선경로로 움직입니다.

```python
from nrmkindy.script import *

if __name__ == '__main__':
    config_script(NAME_INDY_7)
    start_script()

    home()
    movel(TaskPos(x=0.2), mode=MoveBaseMode.RELATIVE)

    mwait_done()
    end_script()
```


## Let's Run!

```shell
$ python3 quick_start.py
```


## What's Next

[Tutorial](Tutorial.md)을 따라하면서 IndyScript 사용법을 차례대로 익혀보거나, [Reference 가이드](Reference.md)를 보고 여러분만의 로봇 스크립트를 만들어 보세요!