import threading
import atexit
import multiprocessing
import time

class C2:
    def __init__(self):
        self.name = 'LaWanda'
        self.processes = []
        self.processes.append(multiprocessing.Process(target=self.idle,name = 'karen'))
        self.processes.append(multiprocessing.Process(target=self.verify,name = 'joshua'))
        atexit.register(self.shutdown)
        for i in self.processes:
            i.start()
        # while(True):
        #     print('hahahaha you can see this? ')
    def idle(self):
        while(True):
            print('idling C2')
            time.sleep(5)
    def verify(self):
        while(True):
            time.sleep(5)
            print('verifying C2')
    def shutdown(self):
        print('shutting the whole circus down')
        for i in self.processes:
            i.terminate()
    def return_processes(self):
        return self.processes
