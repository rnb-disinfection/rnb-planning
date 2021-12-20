# Demo process

## Initial setup
1. Turn On the robot.
2. Connect WIfi to Kiro_MobileRobotNo2_5G (no-internet local network)
3. Connect to Planning PC with VNC viewer (192.168.0.10, pw:0000)
4. In VNC viewer, open terminal and start jupyter
```bash
jupyter notebook
```
  * Now you can connect to jupyter notebook at 192.168.0.10:8888
5. Running IndySDK
  - Connect to Indy with MobaXterm (192.168.0.3, kiro-ros / 0000)
  - Start IndySDK TaskManager
  ```bash
  sudo killall -9 UDEVMonitor && sudo killall -9 TaskMan \
  && cd release/IndySDK_2.3.0.1/ \
  && sudo ./TaskManager -j indyDeploy.json
  ```
  - **NOTE** Only trajectory client is used in this demo. ControlHub UI is not accessable.
6. Connect Conty and reset error state (version2.3.0.1)
  * if version does not match, get conty apk files from http://192.168.0.10:8888/tree/Downloads (It may be already downloaded on tablet.)

## Running
1. open and run script *demo_1222_Load_and_Execute.ipynb*