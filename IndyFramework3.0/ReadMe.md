# Install Indy Framework 3.0  
* Put Framework 3.0 task manager on /home/user/release/IndyFramework3.0/ on STEP PC  
* ./TaskManager -j indyDeploy.json  
* Install NRMK Framework and SDK on Host PC  
* Update indy model to latest version on "C:\Program Files (x86)\neuromeka\NRMKFoundation\bin\models\Indy7"  
* Launch NRMKFoundation, open CadKit  
* Import the updated indy model  
* Connect to STEP PC  
* Check operation (D -> 1, 2)  

# Setup GRPC  
* Install toolchain & libraries for Framework 3.0 on STEP PC
* fix_stdlibc++.sh is attached, run it on STEP PC
* Follow instruction on INSTALL_GRPC.md
* If error occurs on apt-get update, upload key
  * sudo apt-key adv keyserver --keyserver.ubuntu.com --recv-keys \<your key, on error message NO_PUBKY **XXXXX**\>