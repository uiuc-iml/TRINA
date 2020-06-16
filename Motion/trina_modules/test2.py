import threading
import atexit
import multiprocessing
import time


class C2:
    def __init__(self,jarvis):
        from reem.connection import RedisInterface
        from reem.datatypes import KeyValueStore
        from matplotlib import pyplot as plt

        self.name = 'LaWanda'
        self.processes = []
        self.jarvis = jarvis
        # while(True):
        #     a = self.jarvis.get_rgbd_images()
        #     print('got Images! 2',a['realsense_right'][0].shape)
        #     plt.imshow(a['realsense_right'][0])
        #     plt.show()
        # self.processes.append(multiprocessing.Process(target=self.idle,name = 'karen',args = self.jarvis))
        # self.processes.append(multiprocessing.Process(target=self.verify,name = 'joshua', args = self.jarvis))
        self.processes.append(threading.Thread(target=self.idle,name = 'karen'))
        self.processes.append(threading.Thread(target=self.verify,name = 'joshua'))
        self.interface = RedisInterface(host="localhost")
        self.interface.initialize()
        self.server = KeyValueStore(self.interface)
        self.sleep_time = 5
        atexit.register(self.shutdown)
        for i in self.processes:
            i.start()
        # while(True):
        #     print('hahahaha you can see this? ')
    def idle(self):
        from matplotlib import pyplot as plt

        time.sleep(1)
        while(True):
            # print('idling C2')
            time.sleep(self.sleep_time)
            a = self.jarvis.get_rgbd_images()
            # print('got Images! 2',a['realsense_right'][0].shape)
            # plt.imshow(a['realsense_right'][0])
            # plt.show()
    def verify(self):
        while(True):
            try:
                self.server['health_log']['C2'] = [True,time.time()]
                time.sleep(1)
            except Exception as e:
                print(e)
    def shutdown(self):
        print('shutting the whole circus down')
        for i in self.processes:
            i.terminate()
    def return_processes(self):
        return self.processes
