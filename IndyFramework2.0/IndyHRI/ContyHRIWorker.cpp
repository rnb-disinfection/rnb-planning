/*
 * ContyHRIWorker.cpp
 *
 *  Created on: 2018. 1. 11.
 *      Author: Hanter Jung
 */

#include "ContyHRIWorker.hpp"

#define x_USE_ADB_KILL_SERVER

namespace NRMKIndy
{
namespace HRI
{

ContyHRIWorker::ContyHRIWorker(IndyController * controller, SYSTEM_INTERFACE * sysInterface, NRMKFramework::ShmemManager * indyShmem)
: HRIWorker(controller)
, _sendDataIntervalAdb(DATA_COMM_ADB_DEFAULT_INTERVAL)
, _sendDataIntervalTcp(DATA_COMM_TCP_DEFAULT_INTERVAL)
, _connected(false)
, _eventGenerator(Events::EventGenerator::getInstance())
, _indyShmem(indyShmem)
, _contyAdbSktIdx(0)
, _contyAdbIPAddr(const_cast<char*>("127.0.0.1"))
, _contyAdbPorts {6002, 6003, 6004}
, _adbConnectionCheckingCount(0)
{
	_contyTcpSocket.setIndyShmem(indyShmem);
	_contyTcpSocket.setController(controller);
	_contyTcpSocket.setSysInterface(sysInterface);

	_contyAdbSocket.setIndyShmem(indyShmem);
	_contyAdbSocket.setController(controller);
	_contyAdbSocket.setSysInterface(sysInterface);
}

ContyHRIWorker::~ContyHRIWorker()
{
	if (_isWorking) stop();
//#if defined (_USE_ADB_KILL_SERVER)
	system("sudo adb kill-server");
//#endif
}

void ContyHRIWorker::setWifiSendDataInterval(int interval)
{
	if (interval < 10) interval = 10;	//minimum 10ms
	if (interval > 1000) interval = 1000;	//maximum 1s
	_sendDataIntervalTcp = interval;
}

void ContyHRIWorker::setAdbSendDataInterval(int interval)
{
	if (interval < 33) interval = 33;	//minimum 33ms
	if (interval > 1000) interval = 1000;	//maximum 1s
	_sendDataIntervalAdb = interval;
}

void ContyHRIWorker::sendAddCurrentWaypoint(bool callInRealtime)
{
	if (callInRealtime)
	{
		ContyHRICommand cmd;
		cmd.cmdType = CONTY_HRI_CMD_ADD_CURRENT_WAYPOINT;

		//TODO call in realtime do NOT USE std:queue
		_hriCmdQueue.push(cmd);
	}
	else _sendAddCurrentWaypoint();
}

bool ContyHRIWorker::_sendAddCurrentWaypoint()
{
	if (_contyTcpSocket.hasConnection() && _connected)
	{
		return _contyTcpSocket.sendAddWaypoint();
	}
	else if (_contyAdbSocket.hasConnection() && _connected)
	{
		return _contyAdbSocket.sendAddWaypoint();
	}
	else return false;
}

void ContyHRIWorker::sendToolStateUpdated(int toolId, bool callInRealtime)
{
	if (callInRealtime)
	{
		ContyHRICommand cmd;
		cmd.cmdType = CONTY_HRI_CMD_UPDATE_TOOLSTATE;
		cmd.argsInt[0] = toolId;

		_hriCmdQueue.push(cmd);
	}
	else _sendToolStateUpdated(toolId);
}

bool ContyHRIWorker::_sendToolStateUpdated(int toolId)
{
	if (_contyTcpSocket.hasConnection() && _connected)
	{
		//FIXME toolIndex
		return _contyTcpSocket.sendUpdateToolState(toolId, toolId);
	}
	else if (_contyAdbSocket.hasConnection() && _connected)
	{
		return _contyAdbSocket.sendUpdateToolState(toolId, toolId);
	}
	else return false;
}

void ContyHRIWorker::startEmodiImpedaceCtrl()
{
#if defined (_USE_LiModi) && defined (_IMPEDANCE_CONTROL)
	if (_controller->cmode() == INDY_CMODE_IMPEDANCECTRL)
	{
		if (_contyTcpSocket.hasConnection() && _connected)
		{
			_controller->enableImpedanceLock(_contyTcpSocket.getEmodiImpedanceUnlockAxes());
		}
		else if (_contyAdbSocket.hasConnection() && _connected)
		{
			_controller->enableImpedanceLock(_contyAdbSocket.getEmodiImpedanceUnlockAxes());
		}
	}
#endif
}

void ContyHRIWorker::stopEmodiImpedaceCtrl()
{
#if defined (_USE_LiModi) && defined (_IMPEDANCE_CONTROL)
	// _controller->resetFTBias() = true;
	_controller->enableImpedanceLock(NULL);

	// _controller->stopSlow();
#endif
}

void ContyHRIWorker::_sendNextHRICommand()
{
	if (_hriCmdQueue.size() > 0)
	{
		const ContyHRICommand hriCmd = _hriCmdQueue.front();

		switch (hriCmd.cmdType) {
		case CONTY_HRI_CMD_ADD_CURRENT_WAYPOINT:
			if (_sendAddCurrentWaypoint())
				_hriCmdQueue.pop();
			break;

		case CONTY_HRI_CMD_UPDATE_TOOLSTATE:
			if (_sendToolStateUpdated(hriCmd.argsInt[0]))
				_hriCmdQueue.pop();
			break;

		default:
			_hriCmdQueue.pop();
			return;
		}
	}
}


void ContyHRIWorker::run()
{
	ProgramManager & progManager = ProgramManager::getInstance();

	while (_isWorking && !_isFQuit)
	{
		if (_isPaused)
		{
			Poco::Thread::sleep(100);
			continue;
		}
		else
		{
			if (_contyTcpSocket.hasConnection())
			{
				if (!_connected)	//On Connection
				{
					_connected = true;

					Events::Event contyConnectedEvent;
					contyConnectedEvent.set(Events::EventCategory::CAT_ROBOT_STATUS, Events::EventType::ROBOT_STATE_UPDATED,
							_controller->time(), NRMKIndy::RobotStateType::STATE_CONTY_CONNECTION, 1);
					_eventGenerator.publishEvent(contyConnectedEvent);

					_tcpMode = true;
#if defined (_USE_ADB_KILL_SERVER)
					system("adb kill-server");
#endif

					printf("ContyHRIWorker : Connected with Conty via TCP!\n");
				}

				_contyTcpSocket.sendUpdateCurrentData();
				_sendNextHRICommand();

				Poco::Thread::sleep(_sendDataIntervalTcp);
			}
			else if (_contyAdbSocket.hasConnection())
			{
				if (!_connected)	//On Connection
				{
					_connected = true;

					Events::Event contyConnectedEvent;
					contyConnectedEvent.set(Events::EventCategory::CAT_ROBOT_STATUS, Events::EventType::ROBOT_STATE_UPDATED,
							_controller->time(), NRMKIndy::RobotStateType::STATE_CONTY_CONNECTION, 1);
					_eventGenerator.publishEvent(contyConnectedEvent);

					_tcpMode = false;
					//FIXME stop and restart failed...
					//When adb socket is connected, stop the TCP server
//					printf("ContyHRIWorker : Temporary stop TCP server!\n");
//					_contyTcpSocket.CloseComm();
//					_contyTcpSocket.StopComm();

					printf("ContyHRIWorker : Connected with Conty via ADB!\n");
				}

				_contyAdbSocket.sendUpdateCurrentData();
				_sendNextHRICommand();

				Poco::Thread::sleep(_sendDataIntervalAdb);
			}
			else	//no connection
			{
				if (_connected)		//On Disconnection
				{
					_connected = false;

					Events::Event contyDisconnEvent;
					contyDisconnEvent.set(Events::EventCategory::CAT_ROBOT_STATUS, Events::EventType::ROBOT_STATE_UPDATED,
							_controller->time(), NRMKIndy::RobotStateType::STATE_CONTY_CONNECTION, 0);
					_eventGenerator.publishEvent(contyDisconnEvent);
					printf("ContyHRIWorker : Disconnected from Conty!\n");

					if (_tcpMode)	//TCP Server disconnected
					{
						//do nothing
					}
					else	//ADB connection disconnected
					{
						//Run again TCP server
						//FIXME stop and restart failed...
//						printf("ContyHRIWorker : Run TCP server again!\n");
//						if (_contyTcpSocket.startServer(SOCK_TCP, 6001))
//							printf("Conty HRI TCP Server started at IP of : %s on Port: %d\n", _contyTcpSocket.getAddress(), 6001);
//						_contyTcpSocket.waitForConnection(0);
					}

					_adbConnectionCheckingCount = 0;
				}

				//FIXME When socket connection lost, terminate a current program if the program is stopped
				/*if (progManager.getCurrentProgram() != NULL)
				{
					if (progManager.programMode() == NRMKIndy::HRI::PROG_MODE_CONTY &&
							progManager.getCurrentProgram()->getRunningState() == NRMKIndy::HRI::PROG_STATE_STOPPED)
					{
						progManager.releaseCurrentProgram();
					}
				}*/

				if (_adbConnectionCheckingCount < 30)
				{
					Poco::Thread::sleep(100);
					_adbConnectionCheckingCount++;
					continue;
				}
				else	//3000ms to check adb connection
				{
					_adbConnectionCheckingCount = 0;
				}

				//ADB Socket
				FILE * adbFp = NULL;
				char adbStrBuff[256];
				char retBuff[256];

				// printf("Reconnecting to Conty [port=%d]\n", _contyAdbPorts[_contyAdbSktIdx]);
#if 0
#if defined (_USE_ADB_KILL_SERVER)
				system("sudo adb kill-server");
#endif
				sprintf(adbStrBuff, "sudo adb forward tcp:%d tcp:%d", _contyAdbPorts[_contyAdbSktIdx], _contyAdbPorts[_contyAdbSktIdx]);
				system(adbStrBuff);

#else			//USE PIPE
//				printf("ContyHRIWorker : Reconnecting to Conty via ADB using pipe\n");
				//Kill server for reconnecting adb
#if defined (_USE_ADB_KILL_SERVER)
				system("adb kill-server");
#endif

				if (_contyTcpSocket.hasConnection()) continue;	//just check again

                system("adb devices > adb_devices");
                std::ifstream fin("adb_devices");

                if (!fin)
                {
//                    printf("ContyHRIWorker : Getting ADB deivces failed.\n");
                    _adbConnectionCheckingCount = 0;
                    continue;
                }
                bool isADBError = false;
                bool hasDevices = false;

                const char* retBuffPtr;
                std::string myBuff;
                while (!fin.eof() && getline(fin, myBuff)) {
                    retBuffPtr = myBuff.c_str();
//                    cout << retBuffPtr << endl;
                    if (retBuffPtr[0] == '*') continue; //annotation
                    else if (strstr(retBuffPtr, "error") != nullptr) {
                        isADBError = true;
//                      printf("ContyHRIWorker : ADB Error - %s\n", retBuffPtr);
                        break; //error
                    } else if (strstr(retBuffPtr, "List of") != nullptr) continue; //list header
                    else if (strstr(retBuffPtr, "device") != nullptr) {
                        hasDevices = true;
                    }
                }
//                cout << isADBError << "," << hasDevices << endl;

                fin.close();
                system("rm adb_devices");

				if (_contyTcpSocket.hasConnection()) continue;	//just check again

				if (isADBError)
				{
					_adbConnectionCheckingCount = 0;
					continue;
				}

                if (hasDevices)
                {
                    sprintf(adbStrBuff, "adb forward tcp:%d tcp:%d > adb_forward", _contyAdbPorts[_contyAdbSktIdx], _contyAdbPorts[_contyAdbSktIdx]); //without sudo (because program already run with root auth)
                    system(adbStrBuff);
                    std::ifstream finForward("adb_forward");
                    if(!finForward)
                    {
                        // printf("ContyHRIWorker : ADB Forwarding Failed.\n");
                        _adbConnectionCheckingCount = 0;
                        continue;
                    }
                    else
                    {
                        // printf("ContyHRIWorker : ADB Forwarded!\n");
                        finForward.close();
                        system("rm adb_forward");
                    }
                }
				else
				{
//					printf("ContyHRIWorker : No devices.\n");
					_adbConnectionCheckingCount = 0;
					continue;
				}
#endif

				if (_contyTcpSocket.hasConnection()) continue;	//just check again

				//try connect via adb
				if (_contyAdbSocket.connect(_contyAdbIPAddr, _contyAdbPorts[_contyAdbSktIdx]) == 0)
				{
#if defined (GUISOCKET_USE_O)
					_contyAdbSocket.setConnected();
#endif
					printf("ContyHRIWorker : Connected with Conty via ADB tcp=%d!\n", _contyAdbPorts[_contyAdbSktIdx]);
					_contyAdbSktIdx = (_contyAdbSktIdx + 1) % 3;
#if not defined (GUISOCKET_USE_O)
//					Poco::Thread::sleep(1500);
					for (int i=0; i<15; i++)
					{
						if (_contyTcpSocket.hasConnection()) Poco::Thread::sleep(100);	//just check again (adb connection alive)
						else
						{
							printf("ContyHRIWorker : ADB is not connected with Conty!\n");
							break;
						}
					}
#else
					for (int i=0; i<15; i++)
					{
						if (_contyTcpSocket.hasConnection()) break;	//just check again
						else Poco::Thread::sleep(100);
					}
#endif
				}
				else
				{
					_contyAdbSktIdx = (_contyAdbSktIdx + 1) % 3;
					_adbConnectionCheckingCount = 0;
				}
			}
		}
	}
}

bool ContyHRIWorker::start()
{
	if (_isWorking) return false;

	if (_contyTcpSocket.startServer(SOCK_TCP, 6001))
	{
		printf("ContyHRIWorker : Conty HRI TCP Server started at IP of '%s' on Port '%d'\n", _contyTcpSocket.getAddress(), 6001);
		_contyTcpSocket.waitForConnection(0);
		printf("ContyHRIWorker : Conty HRI TCP Server waiting...!\n");
	}
	else
	{
		printf("ContyHRIWorker: Conty HRI TCP Server cannot be started.\n");
		return false;
	}

	_isPaused = false;
	_isWorking = true;
	_thread.start(*this);
	_thread.setOSPriority(74, SCHED_FIFO);
	return true;
}

bool ContyHRIWorker::stop()
{
	if (_isWorking)
	{
		_isWorking = false;
		_isPaused = false;

		_contyTcpSocket.CloseComm();
		_contyAdbSocket.CloseComm();
		_contyTcpSocket.StopComm();
		_contyAdbSocket.StopComm();

		if (_thread.isRunning())
			_thread.join();
		_connected = false;
	}
	else
	{
		_contyTcpSocket.CloseComm();
		_contyTcpSocket.StopComm();
	}
	return true;
}

bool ContyHRIWorker::pause()
{
	if (!_isWorking) return false;
	_isPaused = true;
	return true;
}

bool ContyHRIWorker::resume()
{
	if (!_isWorking) return false;
	_isPaused = false;
	return true;
}


} /* namespace HRI */
} /* namespace NRMKIndy */
