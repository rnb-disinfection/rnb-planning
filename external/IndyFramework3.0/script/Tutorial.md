# 2. Tutorial

본 튜토리얼을 통해 뉴로메카의 IndyScript을 사용하여 로봇 스크립트 프로그래밍 하는 방법을 배울 수 있습니다.

IndyScript는 Python 기반 (Python 3.7.x)의 로봇 프로그래밍 스크립트로, 뉴로메카의 IndyFramework 3.0에서 로봇 제어 등의 다양한 기능을 제공합니다. 본 튜토리얼에서는 Python 문법에 대해서는 설명하지 않으므로, Python에 대하여 별도의 학습이 필요할 수 있습니다. 만약 Python의 문법을 이미 알고 있다면, IndyScript를 쉽게 익힐 수 있습니다.


## Template

별첨된 `template.py` 를 열면 다음과 같은 코드를 볼 수 있습니다.

```python
from nrmkindy.script import *

if __name__ == '__main__':
    config_script(NAME_INDY_7)
    start_script()

    # Place Your CODE

    mwait_done()
    end_script()

```

`config_script()`는 스크립트를 시작하기에 앞서 로봇 모델과 함께 초기화 해주는 함수(function)입니다.

`start_script()` 구문 부터 `end_script()`구문 까지는 실제 로봇 스크립트 프로그래밍이 동작하는 영역입니다.

`mwait_done()`은 로봇의 현재 모션이 끝날 때까지 대기하는 함수입니다.

이렇게 IndyScript는 **항상 위 템플릿을 기반으로 작성**됩니다. 코드 중간에 주석으로 표현된 *`# Place Your CODE`* 부분을 지우고 아래 지침을 따라 첫 스크립트를 작성해보세요.


## Position

로봇을 원하는 위치로 이동하려면 목표위치를 입력해야 하며, 이러한 로봇의 위치는 관절공간(Joint Space)에서의 위치와, 작업공간(Task Space)에서의 위치로 두 종류가 있습니다. 각 위치는 `JointPos`와 `TaskPos`로 표현할 수 있으며, `list`나 `tuple`로도 표현이 가능합니다.

```python
pos1 = JointPos(45, 0, -90, 0, -90, 30)
pos2 = TaskPos(0.55, -0.18, 0.62, 0, -180, 0)
```

`JointPos`는 **관절공간에서 위치를 나타내는 클래스**입니다. 관절공간 상의 위치는 각 축의 회전각도로 표현됩니다. Indy7의 경우, 6축 다관절 로봇이므로 `JointPos`의 인자는 총 6개이며, 각각의 인자값들은 차례대로 로봇의 첫번째 관절부터 마지막 관절까지의 회전각도를 의미합니다. 이 때, 단위는 Degree 입니다. (이하 별도의 언급이 없는 경우 문서 상의 모든 기본 각도 단위는 Degree를 의미합니다)

`TaskPos`는 로봇의 **작업공간에서 위치를 나타내는 클래스**입니다. 작업공간 상의 위치는 6자유도이므로 `TaskPos`의 인자는 6개이며, 각각의 인자값들은 차례대로 공간 상의 위치 x, y, z와 공간 상의 회전 u, v, w를 의미합니다. 이 때, 공간 상의 위치를 나타내는 x, y, z는 각각 기준 좌표계의 원점에서부터 x축, y축, z축만큼 떨어진 거리이며, 단위는 meter 입니다. 이와 함께 공간 상의 회전 u, v, w는 각각 기준 좌표계의 x축, y축, z축의 회전순서로 정의되는 회전각도입니다. (이러한 회전 정의를 fixed XYZ라고도 합니다.)

`JointPos`나 `TaskPos`는 여러 값들을 차례대로 나열한 데이터 컨테이너로 볼 수 있습니다. 따라서 클래스를 사용하지 않고 (클래스를 사용하는 이유는 해당 좌표 데이터를 명확하게 하기 위함입니다) `list`나 `tuple`을 이용하여 다음과 같이 간략히 표현할 수도 있습니다.

```python
pos1 = (45, 0, -90, 0, -90, 30)
pos2 = [0, 0, 0.2, 0, 0, 90]
```

#### 로봇의 현재위치 받아오기

로봇의 현재위치를 로봇 스크립트에서 자유롭게 사용할 수 있다면 프로그램을 더 원활히 개발할 수 있습니다. 이러한 현재위치는 콘티(Conty)를 통해 직접 확인하거나 또는 미리 티칭된 값을 사용할 수도 있겠지만, 이러한 로봇 위치를 스크립트에서 직접 사용하고 싶은 경우 `getpos.py`를 활용합니다. 이 스크립트를 실행함과 동시에 `q`, `p`, `pb` 인자를 함께 입력하면 각각 현재 로봇의 관절공간위치(Joint Space Position)와 참조 좌표계(Reference Frame) 기준의 작업공간위치(Task Space Position), 그리고 로봇베이스 좌표계(Robot Base Frame) 기준의 작업공간위치를 받아올 수 있습니다.

```shell
$ python3 getpos.py q           # 현재 로봇의 관절공간위치
$ python3 getpos.py p           # 참조 좌표계에서 바라본 현재 로봇의 작업공간위치
$ python3 getpos.py pb          # 로봇베이스 좌표계에서 바라본 현재 로봇의 작업공간위치
```


## Motion

이제 로봇을 움직여볼 차례입니다. 로봇이 **특정 경로를 따라 로봇이 목표위치로 이동하는 것을 모션**(Motion)이라고 합니다. 여기서는 다음과 같이 관절공간과 작업공간에서의 가장 기본적인 모션을 사용해 로봇을 움직여 보겠습니다.

```python
movej(pos1)
movel(pos2)
```

위의 예제 스크립트를 실행하면 로봇이 `pos1` 위치까지 임의의 경로로 움직인 후 정지한 다음, 다시 `pos2` 위치까지 직선 경로로 움직입니다.

`movej()`는 **관절공간에서의 모션**으로, 각 관절의 회전축을 중심으로 목표위치인 회전각도만큼 각 관절들이 동시에 회전을 시작하고 동시에 정지합니다. 따라서 로봇이 공간 상에서 어떤 경로로 움직일지 예측할 수 없지만, 특이점(Singularity)과 상관없이 로봇의 모든 작업영역에서 이동이 가능합니다. 이러한, `movej()` 함수의 인자는 `JointPos` 객체나 관절위치가 입력된 `list`, `tuple`의 형태이어야 합니다.

`movel()`은 **작업공간에서의 모션**으로, 공간 상에서 기준 좌표계를 기준으로 목표위치인 각 축 기준의 이동거리와 회전각도만큼 로봇 TCP(Tool Center Point)를 최단거리로 움직입니다. 따라서 `movej()`와 달리 로봇이 어떤 경로로 움직일지 예측 가능하지만, 이동 경로에 따라 특이점이 존재할 수 있어 로봇의 모든 작업영역에서의 이동이 제한적일 수 있습니다. 또한 기준 좌표계에 따라 표시되는 목표위치값이 상이하므로, 사용 시 좌표계에 대해 충분히 숙지하고 사용하는 것이 필요합니다. 이러한 `movel()` 함수의 인자는 `TaskPos` 객체나 공간위치가 입력된 `list`, `tuple`의 형태이어야 합니다.  

참고로 모션 함수의 위치 인자들은 위의 예제와 같이 미리 좌표 변수로 설정하는 방법 외에도 아래와 같이 바로 위치값을 입력하는 방법도 가능합니다.

```python
movej([45, 0, -90, 0, -90, 30])
movej(JointPos(45, 0, -90, 0, -90, 30))
```


## Relative Motion

앞서 설명한 `movej()`와 `movel()`은 **현재위치에 상관없이 입력된 위치값까지 로봇이 이동**하게 되는데, 이를 절대위치로의 이동 또는 **절대모션**이라고 부릅니다. 

반면, **현재 위치를 기준으로 입력된 위치값만큼 로봇을 이동**시키는 경우를 상대위치로의 이동 또는 **상대모션**이라고 합니다. 이러한 상대모션은 기존 절대모션을 나타내는 모션 함수명에 relative의 앞자 r을 붙혀 `rmovej()` 또는 `rmovel()`과 같은 함수명을 통해 관련기능을 함께 제공하고 있습니다. 이 때, 함수의 위치 인자인 `JointPos` 또는 `TaskPos`에는 현재 위치를 기준으로 움직일 상대위치를 입력합니다. 다른 방법으로, named parameter를 통해 필요한 인자에만 값을 입력할 수도 있습니다. 

```python
# 상대위치 정의
pos3 = JointPos(60, 0, 0, 0, 0, 0)      # 현재 관절공간좌표로부터 1번째 축이 60만큼 움직임
pos4 = TaskPos(0.1, 0, 0.1, 0, 0, 0)    # 현재 작업공간좌표로부터 x축으로 0.1m, y축으로 0.1만큼 움직임

# named parameter를 이용한 상대위치 정의
pos3 = JointPos(q0=60)         # same as JointPos(60, 0, 0, 0, 0, 0) 
pos4 = TaksPos(x=0.1, z=0.1)   # same as TaskPos(0.1, 0, 0.1, 0, 0, 0)
```

`JointPos`의 named parameter는 `q0`부터 `q5`까지며(Indy7; 관절 개수가 모두 6개인 경우), 각각 1번 관절부터 6번 관절까지를 의미합니다(프로그래밍에서 인덱스는 0부터 시작되므로 q0은 1번축, q1는 2번축을 의미합니다). `TaskPos`의 named paramter는 각 원소 값에 맞게 `x` `y` `z` `u` `v` `w` 가 있습니다.

따라서 아래와 같은 예제의 경우, 로봇 TCP는 현재 위치를 기준으로 z축으로 0.1미터만큼 이동하면서 동시에 z축을 기준으로 90도만큼 회전할 것입니다.

```python
rmovel(TaskPos(z=0.1, w=90))
```


## Waiting Motion Done (Only for Alpha)

`mwait_done()`은 앞서 실행된 모션(들)이 완료될 때 까지 다음 명령어 실행을 기다리게 하는 용도로 사용합니다.

참고로 IndyFramework 3.0에는 모션이 완료된 후 다음 명령어를 실행하게 하는 동기 모션과 반대로 모션 함수 실행과 함께 바로 다음 명령어를 실행하게 하는 비동기 모션이 제공되고 있습니다. 다만, alpha 버전에 한해서 모션 명령어 바로 다음에 모션 명령어가 아닌 다른 명령어가 사용될 경우 모션의 동기 여부와 상관없이 바로 다음 명령어가 실행되고 있습니다. 따라서, 동기 모션 다음에 프로그램 흐름이 변경되는 구문(if, else 등)이 호출되기 전이나 동기모션 사이에 다른 작업(함수 호출, I/O 등)들이 필요할 때, 반드시 `mwait_done()` 함수를 사용해 **모션이 끝날 때까지 다음 스크립트 진행을 대기**시켜야 합니다. 이는 beta 버전 이후에는 불필요한 과정이지만, 현재는 꼭 다음의 경우에 대해서 `mwait_done()`을 호출하여 원치 않는 시점에 프로그램이 진행되는 것을 방지해야합니다.

#### Conditional Statements

`if`/`elif`/`else`와 같은 조건문 전에 동기모션 사용 시, 조건문 전에 `mwait_done()` 사용해야합니다.

```python
movej(pos1)

mwait_done()
if ...:
    movej(pos2)
else ...:
    movej(pos3)
```

#### Looping Statements

`for`와 같은 반복문 안에 동기모션 사용 시, 반복문 시작 전, 그리고 반복문 마지막에 `mwait_done()` 사용해야합니다.

```python
movej(pos1)

mwait_done()
for ...:
    movej(pos2)
    movej(pos3)
    mwait_done()
```

#### Etc.

모션 사이에 함수 호출, I/O 등 다른 작업을 해야 할 경우 반드시 mwait_done()을 사용해야합니다. (I/O는 다음 장에서 배웁니다.)

```python
movel(pos4)
mwait_done()    # 모션 이후 I/O가 오므로 mwait_done() 사용 후 I/O 호출
do(0, HIGH)

movel(pos4)
user_function() # 모션 이후 함수가 오므로 mwait_done() 사용 후 함수 호출
do(0, HIGH)
```

또한 스크립트에서 실행 될 함수를 정의 한 경우, 함수 내에서 모션이 사용된다면 반드시 함수 마지막에 mwait_done()을 사용해야 합니다.

```python
def user_function():
    ...
    movel(pos5)
    ...
    mwait_done()
```


## I/O

신호 입출력 명령어들은 컨트롤박스의 I/O 보드 또는 엔드툴 포트와 전기적으로 연결된 디지털 및 아날로그 신호의 입출력을 제어하는데 사용됩니다.

#### 출력 (Digital Output, Analog Output)

컨트롤박스 또는 엔드툴 포트와 연결된 외부장치로 신호를 전송하기 위한 명령어로서 `do()` 함수와 `ao()` 함수를 제공합니다.

```python
do(1, HIGH, 3, LOW)
do(8, 1, 9, 0)  #same as do(8, HIGH, 9, LOW)
ao(0, 1234)
```

`do()`는 디지털 출력신호를 제어하는 함수로서, 입력 인자는 `1, HIGH`와 `3, LOW`와 같이 두 개의 인자가 쌍으로 함께 입력되어야 합니다. 각 인자의 첫번째 값은 출력신호의 인덱스 ID를 의미하며, 이는 컨트롤박스 IO보드에서 제공되고 있는 디지털 출력신호 채널들에게 `0~23`번이 각각 맵핑되어 있습니다. 각 디지털 출력신호 종류와 채널에 따른 인덱스 ID는 함께 제공되고 있는 레퍼런스를 참고하세요. 각 인자의 두번째 값은 출력신호의 제어값을 의미하며, `HIGH` 또는 `LOW` 값을 갖습니다. 이 때, `HIGH` 대신 숫자 `1`을, `LOW` 대신 숫자 `0`을 사용해도 무방합니다. 따라서 위의 예제 스크립트에서 첫번째 줄의 `do()`는 디지털 출력신호 1번 채널에는 `HIGH`를, 동시에 3번 채널에는 `LOW`를 출력하도록 디지털 신호를 제어합니다. 마찬가지로 두번째 줄의 `do()`는 디지털 출력신호 8번 채널에 `HIGH`, 9번 채널에 `LOW`를 출력합니다.  

`ao()`는 아날로그 출력신호를 제어하는 함수로서, 두 개의 인자를 입력해야 합니다. 여기서, 첫번째 인자는 아날로그 출력신호의 인덱스 ID를 의미하며 0 또는 1의 값을 갖습니다. 두번째 인자는 아날로그 출력신호의 크기로서 0~65535 사이의 값을 가지며, 0은 0V, 65535는 10V에 해당합니다.

위와 같이 함수의 인자에 직접 값을 입력하는 방법 외에도 클래스를 사용하여 객체를 미리 만들어 사용할 수도 있습니다. 이렇게 하면 특정 신호를 여러번 동일하게 출력할 경우 매번 함수와 인자를 직접 입력하지 않아도 되므로 오타 등에 의한 실행 실패나 오동작을 사전에 방지할 수 있습니다.  

다음은 `DIO` 클래스를 사용해 디지털 출력신호를 제어하는 방법입니다.

```python
gripper_on1 = DIO(1, HIGH)
gripper_on2 = DIO(3, LOW)

do(gripper_on1, gripper_on2)
```

또는 `DIOSet` 클래스를 사용해 `do()` 함수처럼 인자를 쌍으로 입력하거나, `DIO` 객체를 여러개 사용해 디지털 출력신호 세트를 사전에 구성해 놓을 수도 있습니다.

```python
# DIOSet 클래스 사용
gripper_on = DIOSet(1, HIGH, 3, LOW)

# DIOSet 클래스와 DIO 객체 사용
gripper_on1 = DIO(1, HIGH)
gripper_on2 = DIO(3, LOW)
gripper_on = DIOSet(gripper_on1, gripper_on2)

do(gripper_on)
```

마찬가지로 `ao()` 함수에서도 `AIO` 클래스와 `AIOSet` 클래스를 사용할 수 있습니다.

#### 입력 (Digital Input, Analog Input)

컨트롤박스 또는 엔드툴 포트와 연결된 외부장치로부터 전송된 신호를 감지하기 위한 명령어로서 `di()` 함수와 `ai()` 함수를 제공합니다. 이러한 입력 함수들은 특정 채널의 인덱스 ID를 인자로 사용하는 출력 함수들과 달리 모든 입력 채널에 해당하는 인덱스의 ID와 감지값을 `tuple` 형태로 한번에 받아 옵니다. 따라서 디지털 입력신호의 경우 컨트롤박스에서 `0~23`번의 인덱스 ID에 매핑되어 제공되고 있으므로 `di()`는 길이 24의 tuple을 리턴하며, 각각의 인덱스 ID에는 각 채널에서 감지한 결과값이 `HIGH` 또는 `LOW`로 담겨 있습니다. 만약, 특정 인덱스 ID에 해당하는 값을 확인하고 싶다면 `[]` 연산자를 사용해 받아올 수 있습니다. 이와 유사하게 아날로그 입력신호의 경우에도 컨트롤박스에서 `0~3`번의 인덱스를 제공하고 있으므로 `ai()`는 길이 4의 tuple을 리턴하며, 각각의 인덱스 ID에는 각 채널에서 감지한 결과값이 0에서 65535 이내의 값으로 담겨 있습니다. 이 때, 0은 0V, 65535는 10V에 해당합니다.  

```python
di()        # 길이 24의 tuple 형태로 모든 디지털 입력신호값을 리턴
ai()[1]     # [] 연산자를 사용해 인덱스 ID 2번에 해당하는 값을 리턴
```

참고로 입력 함수 `di()` 또는 `ai()`로부터 동시에 읽어진 값을 여러번 사용할 목적이라면 각각의 채널마다 매번 함수를 호출하여 인덱스 ID에 접근하는 것 보다 한번에 함수를 호출한 후 하나의 변수에 저장하여 필요 시 접근하는 것이 성능적인 측면에서 더 유리합니다. 다음의 예제에서 첫번째 경우는 `di()`를 한번만 호출하는 반면 두번째 경우는 두번 호출합니다.

```python
# Good Case : 통신을 1번 수행
tools_input = di()
if tools_input[3] and tools_input[4]:
    pass

# Bad Case : 통신을 2번 수행하여 위의 경우보다 느림
if di()[3] and di()[4]:
    pass
```

그러나 신호를 읽어야 하는 시점이 다를 경우 각 시점마다 입력 함수를 사용해야 합니다. 다음의 예제에서 첫번째 경우는 if문이 실행될 때마다 `di()` 함수를 사용하여 그 시점의 입력값을 읽었지만, 두번째 경우는 `if`문 사이에 다른 코드들로 인해 두 번째 `if`문이 실행될 시점의 실제 입력값과 다른 값을 가질 수 있습니다.  

```python
# Good Case : 첫번째와 두번째 if문의 실행 시점이 다르므로, 
#             각 if문에서 di()를 호출하여 입력값을 확인해야 함
if di()[5]:
    pass
# ... Some codes ...
if di()[6]:
    pass

# Bad Case : 두번째 if문의 입력값은 첫번째 if문 시점의 값으로,
#            두 if문 간의 실행시간 간격의 차이가 클수록 의도치 않은 동작이 발생할 수 있음
tools_input = di()
if tools_input[5]:
    pass
# ... Some codes ...
if tools_input[6]:
    pass
```


## Try it out!

이제 배운 것들을 종합하여 스크립트를 작성하고 실행해 볼 시간입니다. 로봇이 관절위치 `[0, 0, -90, 0, -90, 45]`까지 이동 한 뒤, 로봇의 6번 축이 -45도만큼 다시 움직인 후 y축으로 15cm 만큼 직선으로 움직이도록 스크립트를 작성해 보세요. `template.py` 코드를 기반으로 작성하는 것을 잊지 마세요!

스크립트를 다 작성했다면 다음의 예제코드와 비교해 보세요. 만약 아직까지 스스로 작성하기 어렵다면 다음의 예제코드를 참고하여 자신만의 코드를 작성해보길 권유드립니다.

```python
from nrmkindy.script import *

if __name__ == '__main__':
    config_script(NAME_INDY_7)
    start_script()

    target_pos = JointPos(0, 0, -90, 0, -90, 45)

    movej(target_pos)

    rmovej(JointPos(q5=-45))
    rmovel(TaskPos(y=0.15))

    mwait_done()
    end_script()
```

실행해 봅시다.

```shell
$ python3 tutorial.py
```

원하는 데로 로봇이 움직이나요?

#### More Complicated Script

앞의 예제를 완성하는데 문제가 없었다면 I/O를 활용하고, 조건문이나 반복문이 들어간 조금 더 복잡한 스크립트를 자유롭게 작성하고 실험해보세요.

다음 예제코드는 간단한 Pick & Place 공정입니다.

```python
from nrmkindy.script import *

if __name__ == '__main__':
    config_script(NAME_INDY_7)
    start_script()

    tool_on = DIOSet(0, HIGH, 1, HIGH)
    tool_off = DIOSet(0, LOW, 1, LOW)

    home_pos = JointPos(0, 0, -90, 0, -90, 0)
    pick_ready = JointPos(-15, -12, -90, 0, -83, -15)
    place_ready = JointPos(-40, -21, -55, 0, -103, 40)

    movej(home_pos)

    mwait_done()
    for i in range(3):
        movej(pick_ready)
        rmovel(TaskPos(z=-0.15))
        mwait_done()
        do(tool_on)
        rmovel(TaskPos(z=0.15))

        mwait_done()
        if di()[0] == HIGH:
            movej(place_ready)
            rmovel(TaskPos(z=-0.2))
            mwait_done()
            do(tool_off)
            rmovel(TaskPos(z=0.2))
        else:
            do(tool_off)
            movej(home_pos)
            
        mwait_done()

    mwait_done()
    end_script()
```


## Learn More

[Reference 가이드](Reference.md)에서는 더 많은 스크립트 명령어들을 제공하고 있습니다. 비동기 모션, 모션 속성 설정 등 다양한 기능들을 확인해 보세요.
