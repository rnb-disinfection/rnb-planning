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
5. Open demo script at http://192.168.0.10:8888/notebooks/Projects/rnb-planning/src/scripts/demo_202107/demo_1222.ipynb
6. Connect to Indy with MobaXterm (192.168.0.3, kiro-ros / 0000)
7. Start IndySDK TaskManager
```bash
sudo killall -9 UDEVMonitor && sudo killall -9 TaskMan \
&& cd release/IndySDK_2.3.0.1/ \
&& sudo ./TaskManager -j indyDeploy.json
```
7. Connect Conty and reset error state (version2.3.0.1)
  * if version does not match, get conty apk files from http://192.168.0.10:8888/tree/Downloads (It may be already downloaded)
  
8. Set controller to NRIC_Force http://192.168.0.3:9990/

## 