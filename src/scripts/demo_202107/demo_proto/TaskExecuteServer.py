from concurrent import futures
import logging
import math
import time
import grpc
import TaskPlanner_pb2
import TaskPlanner_pb2_grpc
from threading import Thread

class TaskExecutorServicer(TaskPlanner_pb2_grpc.TaskExecutorServicer):
    def __init__(self):
        self.object_info_running = self.get_null_object()
        self.task_finish_flag = False
        
    def mark_task_finished(self, flag=True):
        self.task_finish_flag = flag
        
    def get_null_object(self):
        return TaskPlanner_pb2.object_info(object_id=-1,object_name="",center_x=0,center_y=0)

    def GetRobotInfo(self, request, context):
        return TaskPlanner_pb2.GetRobotInfoResponse(
            width=1.0, depth=1.0, height=1.0, x=0.0, y=0.0)

    def DoDisinfection(self, request, context):
        for object_info in request.object_list:
            self.mark_task_finished(False)
            self.object_info_running.CopyFrom(object_info)
            print("running - {} ({}) at ({:.2}, {:.2})".format(
                object_info.object_name, object_info.object_id, object_info.center_x, object_info.center_y))
            while not self.task_finish_flag:
                time.sleep(0.5)
            self.object_info_running = self.get_null_object()
        return TaskPlanner_pb2.DoDisinfectionResponse(response_flag=1)

    def GetRunningStatus(self, request, context):
        if self.object_info_running is None:
            return TaskPlanner_pb2.RunningStateResponse(object_id=-1)
        else:
            return TaskPlanner_pb2.RunningStateResponse(object_id=self.object_info_running.object_id)
        
def serve(servicer, host='[::]'):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    TaskPlanner_pb2_grpc.add_TaskExecutorServicer_to_server(servicer, server)
    server.add_insecure_port('{}:50307'.format(host))
    server.start()
    server.wait_for_termination()
        
def serve_on_thread(host='[::]'):
    logging.basicConfig()
    servicer = TaskExecutorServicer()
    t = Thread(target=serve, args=(servicer,host))
    t.daemon = True
    t.start()
    return servicer