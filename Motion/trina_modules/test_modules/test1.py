import threading
import atexit
import multiprocessing
import time
import open3d as o3d


class C1:
    def __init__(self,jarvis):
        from reem.connection import RedisInterface
        from reem.datatypes import KeyValueStore
        self.name = 'kristoffer'
        self.processes = []
        self.jarvis = jarvis

        # self.processes.append(multiprocessing.Process(target=self.idle,name = 'bob'))
        # self.processes.append(multiprocessing.Process(target=self.verify,name = 'alice'))
        self.processes.append(threading.Thread(target=self.idle,name = 'bob'))
        self.processes.append(threading.Thread(target=self.verify,name = 'alice'))
        atexit.register(self.shutdown)
        self.interface = RedisInterface(host="localhost")
        self.interface.initialize()
        self.server = KeyValueStore(self.interface)
        self.start_time = time.time()
        self.sleep_time = 10
        for i in self.processes:
            i.start()
        # while(True):
        #     print('hahahaha you can see this? ')
    def idle(self):
        a = threading.Thread(target = self.shout)
        time.sleep(1)
        a.start()
        start_time = time.time()
        while(True):
            now = time.time()
            a = self.jarvis.get_point_clouds()
            time.sleep(self.sleep_time)
            print('got Images! 1',a['realsense_right'])
            # o3d.visualization.draw_geometries([a['realsense_right']])
            # print('waiting to start trouble')
            # if((now-start_time) > 5):
            #     raise Exception('Timeout, punk!')
            
    def shout(self):
        while(True):
            # print('this thread is still active! lalalalalalalalala\n\n')
            time.sleep(self.sleep_time)
    def verify(self):
        while(True):
            try:
                self.jarvis.log_health()
                time.sleep(1)
            except Exception as e:
                print(e)

    def shutdown(self):
        print('shutting the whole circus down')
        for i in self.processes:
            i.terminate()
    def return_processes(self):
        return self.processes
