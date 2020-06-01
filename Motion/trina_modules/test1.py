import threading
import atexit
import multiprocessing
import time

class C1:
    def __init__(self):
        self.name = 'kristoffer'
        self.processes = []
        self.processes.append(multiprocessing.Process(target=self.idle,name = 'bob'))
        self.processes.append(multiprocessing.Process(target=self.verify,name = 'alice'))
        atexit.register(self.shutdown)
        for i in self.processes:
            i.start()
        # while(True):
        #     print('hahahaha you can see this? ')
    def idle(self):
        a = threading.Thread(target = self.shout)
        a.start()
        while(True):
            print('idling C1')
            time.sleep(5)
    def shout(self):
        while(True):
            print('this thread is still active! lalalalalalalalala\n\n')
            time.sleep(4)
    def verify(self):
        while(True):
            time.sleep(5)
            print('verifying C1')
    def shutdown(self):
        print('shutting the whole circus down')
        for i in self.processes:
            i.terminate()
    def return_processes(self):
        return self.processes
