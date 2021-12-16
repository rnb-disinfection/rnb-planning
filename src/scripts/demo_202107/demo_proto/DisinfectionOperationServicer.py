from concurrent import futures
import logging
import math
import time
import grpc
import DisinfectionOperation_pb2
import DisinfectionOperation_pb2_grpc
from threading import Thread

COMMUNICATION_TEST = False

class DisinfectionOperationServicer(DisinfectionOperation_pb2_grpc.DisinfectionOperationServicer):
    def __init__(self):
        self.object_info_running = self.get_null_object()
        self.task_finish_flag = False
        
    def mark_task_finished(self, flag=True):
        self.task_finish_flag = flag
        
    def get_null_object(self):
        return DisinfectionOperation_pb2.DoDisinfectionRequest(object_id=-1,object_name="",center_x=0,center_y=0)

    def DoDisinfection(self, request, context):
        self.mark_task_finished(False)
        object_info = request
        self.object_info_running.CopyFrom(request)
        print("running - {} ({}) at ({:.2}, {:.2})".format(
            request.object_name, request.object_id, request.center_x, request.center_y))
        while not self.task_finish_flag:
            time.sleep(0.5)
            if COMMUNICATION_TEST:
                self.task_finish_flag = True
        self.object_info_running = self.get_null_object()
        return DisinfectionOperation_pb2.DoDisinfectionResponse(response_flag=1)

    def DoDisinfectionComplete(self, request, context):
        assert request.request_flag == 1, \
            "Unexpected DoDisinfectionComplete request flag: {}".format(request.request_flag)
        if self.object_info_running is None:
            return DisinfectionOperation_pb2.DoDisinfectionCompleteResponse(response_flag=0)
        else:
            return DisinfectionOperation_pb2.DoDisinfectionCompleteResponse(response_flag=1)
        
def serve(servicer, host):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    DisinfectionOperation_pb2_grpc.add_DisinfectionOperationServicer_to_server(
        servicer, server)
    server.add_insecure_port(host+':50307')
    server.start()
    server.wait_for_termination()
        
def serve_on_thread(host='[::]'):
    logging.basicConfig()
    servicer = DisinfectionOperationServicer()
    t = Thread(target=serve, args=(servicer,host))
    t.daemon = True
    t.start()
    return servicer