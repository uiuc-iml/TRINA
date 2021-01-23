import pandas as pd
import redis
from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore
import time
from threading import Thread, Lock
from copy import deepcopy, copy
from datetime import datetime
import os
import numpy as np
import atexit
from multiprocessing import Process, Lock
from PIL import Image
import sys
from glob import glob
import cv2
from joblib import Parallel,delayed

if(sys.version_info[0] < 3):
    from StringIO import StringIO 
else:
    from io import StringIO

import pdb

class TrinaQueueReader(object):
    def __init__(self, host='localhost', port=6379):
        self.r = redis.Redis(host=host, port=port)

    def read(self, key):
        with self.r.pipeline() as pipe:
            times = self.r.llen(key)
            for i in range(times):
                pipe.lpop(key)
            res = pipe.execute()
        return res


def save_image_array(array,directory,number):
    if(sys.version_info[0] < 3):
        buf = StringIO()
                        
        if(array.dtype == np.uint8):
            name = directory + '/{}.png'.format(number)
            Image.fromarray(array).save(name,"PNG")
        
        elif(array.dtype == np.float32):
            name = directory + '/{}.png'.format(number)
            array =(1000*array).astype(np.uint16)
            cv2.imwrite(name,array)

#                 img = Image.fromarray(array)
#                 img.save(name,"PNG")
        else:
            name = directory + '/{}.png'.format(number)
            array =(1000*array).astype(np.uint16)
            cv2.imwrite(name,array)
#                 img = Image.fromarray(array)
#                 img.save(name,'PNG')
    else:
        buf = StringIO()
                        
        if(array.dtype == np.uint8):
            # print('using simple save')

            name = directory + '/{}.jpg'.format(number)
            cv2.imwrite(name,array)
        
        elif(array.dtype == np.float32):
            # print('converting first')

            name = directory + '/{}.png'.format(number)
            array =(1000*array).astype(np.uint16)
            cv2.imwrite(name,array)

                # img = Image.fromarray(array)
                # img.save(name,"PNG")
        elif(array.dtype == np.uint16):
            # print('No need to convert')

            name = directory + '/{}.png'.format(number)
            array =(1000*array).astype(np.uint16)
            cv2.imwrite(name,array)
        else:
            print('invalid dtype!')
    
def dump_dataset_to_memory(datasets,copy_images,copy_image_times):
    import pandas as pd
    # print('\n\n\n\n\n\n adding new pictures to log \n\n\n\n\n\n\n')
    # start_time = time.time()
    # print(copy_images.keys())
    try:
        # for key in copy_images.keys():
        #     # print(key)
        #     dset_file = datasets[key]
        #     images = copy_images[key]
        #     times = copy_image_times[key]
        #     # print('started saving images')
        #     save_arrays_as_images(images,dset_file,times)
        Parallel(n_jobs = 4,backend = 'threading')(delayed(save_arrays_as_images)(copy_images[key],datasets[key],copy_image_times[key]) for key in copy_images.keys())
    except Exception as e:
        print(e)
        
#         print('\n\n\n\n preparation of the dataset takes {} seconds'.format(time.time()-start_time))
    # print(f)

    return 0
def save_arrays_as_images(arrays,directory,times):
    # print(directory)
    total = np.max(len(glob(directory+'/*'))-1,0)
    # print(total)
    final = total + len(arrays)
    this_range = range(total,final)
    # print('times df')
    # print(directory)
    # times_df = pd.DataFrame({'trina_time':times})
    # times_df.to_csv(directory+'_times.csv',sep='|',
    #                         mode='a', header=False, index=False)
    if(total == 0):
        with open(directory+'_times.csv','a') as f:
            f.write('times')
    with open(directory+'_times.csv','a') as f:
        f.write('\r\n'+str(times)[1:-1].replace(',','\r\n'))
    # we then save the images to file:
    # print('makes it past times df')
    for i,image in enumerate(arrays):
        # print(i)
        number = total + i
        save_image_array(image,directory,number)




class StateLogger(object):
    def __init__(self, jarvis, frequency=60, image_frequency=15):

        print('\n\n\n starting logger \n\n\n')

        self.states_lock = Lock()
        self.commands_lock = Lock()
        self.images_lock = Lock()
        self.operational_lock = Lock()
        self.comments_lock = Lock()
            
        self.dt = 1.0/frequency
        self.images_dt = 1.0/image_frequency
        self.command_reader = TrinaQueueReader()
        self.jarvis = jarvis

        self.intermediate_wait = 5
        tmp = datetime.now()
        if(not os.path.exists('./logs')):
            os.mkdir('./logs')
        self.name = tmp.strftime("%Y_%m_%d_%H_%M_%S")
        self.jarvis.server['LOGGER_NAME'] = self.name
        self.start_all_logger_variables()
        atexit.register(self.safely_close)
        self.comments = []
        self.comment_times = []
        self.label = ''
        self.logger_status = {'Stop':False,'Pause':False,'Start':True}
        self.jarvis.server['LOGGER_STATUS'] = self.logger_status
        self.logger_items = {'UIState':True,'RobotState':True,'Commands':True,'Video':True}
        self.jarvis.server['LOGGER_ITEMS'] = self.logger_items
        # self.sensors = self.jarvis.sensor_module
        self.stateThread = Thread(target=self.update_states)
        self.commandThread = Thread(target=self.update_command_logs)
        self.saveStateLogThread = Thread(
            target=self.save_state_and_command_log)
        self.updateImageThread = Thread(target=self.update_images)
        self.operational_conditions_thread = Thread(target=self.update_opearational_conditions)
        
        
        self.saveImageLog = Thread(target=self.save_images_to_disk)
        # self.stateThread.daemon = True
        # self.commandThread.daemon = True
        # self.saveStateLogThread.daemon = True
        # self.updateImageThread.daemon = True
        # self.saveImageLog.daemon = True
        # self.operational_conditions_thread.daemon = True
        self.updateImageThread.start()

        self.operational_conditions_thread.start()
        self.stateThread.start()
        self.commandThread.start()
        # time.sleep(3)
        self.saveStateLogThread.start()
        self.saveImageLog.start()

        # self.stateThread.join()            
    
    def start_all_logger_variables(self):
        with self.states_lock:
            with self.commands_lock:
                with self.images_lock:
                    print('\n\n\n starting log with name {} \n\n\n'.format(self.name))
                    # print('\n\n\n {} \n\n\n this'.format(self.name))
                    self.status_filename = "./logs/"+self.name + ".gz"
                    self.images_dir = "./logs/"+self.name+'/'
                    if(not os.path.exists(self.images_dir)):
                        os.mkdir(self.images_dir)
                    self.images = {}
                    self.image_times = {}
                    self.command_list = []
                    self.ui_state = []
                    self.robot_state = []
                    self.ui_times = []
                    self.robot_state_times = []
                    self.dataset_dirs = {}
                    print('first jarvis call for images')
                    tmp = self.jarvis.get_rgbd_images()

                    # tmp = self.jarvis.get_rgbd_images()
                    keys = tmp.keys()
                    print('got keys: {}'.format(keys))

                    self.close_all = False
                    # we start the key dictionaries
                    for key in keys:
                        self.images.update({key+'_color': [], key+'_depth': []})
                        self.image_times.update({key+'_color': [], key+'_depth': []})

                    for key in keys:
                        self.images[key+'_color'].append(tmp[key][0])
                        self.images[key+'_depth'].append(tmp[key][1])
                        self.image_times[key+'_color'].append(tmp[key][2])
                        self.image_times[key+'_depth'].append(tmp[key][2])
#                         print('this is the first time!')
#                         print(tmp[key][2])
                    # we also start the datasets:
                    
                    for dataset_name in tmp.keys():
                        this_dir = self.images_dir+'{}_color'.format(dataset_name)
                        if(not os.path.exists(this_dir)):
                            os.mkdir(this_dir)
                        self.dataset_dirs.update({dataset_name+'_color':this_dir})
                        times_df = pd.DataFrame({'trina_time':[],'image':[]})
                        times_df.to_csv(this_dir+'/times.csv',sep='|',
                                mode='w',index=False)
                           
                    #then for depth
                    for dataset_name in tmp.keys():
                        this_dir = self.images_dir+'{}_depth'.format(dataset_name)
                        if(not os.path.exists(this_dir)):
                            os.mkdir(this_dir)
                        self.dataset_dirs.update({dataset_name+'_depth':this_dir})
                        times_df = pd.DataFrame({'trina_time':[],'image':[]})
                        times_df.to_csv(this_dir+'/times.csv',sep='|',
                                mode='w',index=False)
                print('\n\n Started all the log variables!\n\n')
                    
    def update_states(self):
        while(True):
            start = time.time()
            new_name = self.jarvis.server['LOGGER_NAME'].read()
            if(new_name):
                if(new_name != self.name):
                    self.name = new_name
                    self.start_all_logger_variables()
            with self.states_lock:
                self.ui_state.append(str(self.jarvis.getUIState()))
                self.ui_times.append(str(self.jarvis.getTrinaTime()))
                self.robot_state.append(str(self.jarvis.getRobotState()))
                self.robot_state_times.append(str(self.jarvis.getTrinaTime()))

            elapsed = time.time()-start
            if(elapsed < self.dt):
                time.sleep(self.dt-elapsed)
            if(self.close_all):
                break
    
    def update_opearational_conditions(self):
        while(True):
            start = time.time()
            with self.operational_lock:
                self.logger_status = self.jarvis.server['LOGGER_STATUS'].read()
                self.logger_items  = self.jarvis.server['LOGGER_ITEMS'].read()    
#             print(self.logger_items)
#             print(self.logger_status)
            elapsed = time.time()-start
            if(elapsed < self.dt):
                time.sleep(self.dt-elapsed)
            if(self.close_all):
                break
    
    def update_command_logs(self):
        while(True):
            start = time.time()
            with self.commands_lock:
                new_commands = self.command_reader.read('EXECUTED_COMMANDS')
                if(new_commands):
                    try:
                        if(new_commands[0]):
                            self.command_list.append(new_commands)
                    except:
                        print('error')
            elapsed = time.time()-start
            if(elapsed < self.dt):
                time.sleep(self.dt-elapsed)
            if(self.close_all):
                break

    def yield_states_df(self):
        # print('logging state started')
        with self.states_lock:
            copy_ui_state = copy(self.ui_state)
            copy_robot_state = copy(self.robot_state)
            copy_ui_times = copy(self.ui_times)
            copy_robot_state_times = copy(self.robot_state_times)
            self.ui_state = []
            self.robot_state = []
            self.ui_times = []
            self.robot_state_times = []
        with self.operational_lock:
            if(not self.logger_items['UIState']):
                print('not logging UIState')
                copy_ui_state = []
                copy_ui_times = []
            if(not self.logger_items['RobotState']):
                print('not logging RobotState')
                copy_robot_state = []
                copy_robot_state_times = []
        nature_ui = len(copy_ui_state)*['ui']
        nature_robot = len(copy_robot_state)*['robot']
        logs = copy_ui_state + copy_robot_state
        times = copy_ui_times + copy_robot_state_times
        natures = nature_ui + nature_robot
        states_df = pd.DataFrame(
            {'log': logs, 'trina_time': times, 'nature': natures})

        return states_df

    def yield_commands_df(self):
        # print('yielding the state')
        with self.commands_lock:
            copy_command_list = copy(self.command_list)
            self.command_list = []

        commands = []
        command_times = []
        # print('\n\n\ncommand list is : {}\n\n\n\n'.format(copy_command_list))
        for i in copy_command_list:
            if(i):
                # print('this is i : {}'.format(i))
                try:
                    # tmp = eval(i)
                    # # exec('tmp = {}'.format(str(i)))
                    # # print(tmp)
                    for j in i:
                        # print('\n\nthis is j = {}\n\n'.format(j))
                        tmp2 = eval(j)
                        # print('got to tmp2: {}'.format(tmp2))
                        commands.append(tmp2[0])
                        command_times.append(tmp2[1])
                except Exception as e:
                    print(e)

        if(commands):
            with self.operational_lock:
                if(self.logger_items['Commands']):
                    print('\n\n adding commands!')
                    commands_df = pd.DataFrame(
                        {'log': commands, 'trina_time': command_times})

                    commands_df['nature'] = 'command'

                    commands_df.columns = ['log', 'trina_time', 'nature']
                else:
                    print('\n\n not logging commands \n\n')
        else:
            commands_df = pd.DataFrame(
                {'log': [], 'trina_time': [], 'nature': []})
        return commands_df
    
    def yield_comments_df(self):
        # print('yielding states')
        comments_list = self.command_reader.read('LOGGER_COMMENTS')
        print(comments_list)
        comments = []
        comment_times = []
        for i in comments_list:
            exec('tmp = {}'.format(i))
            for j in tmp:
                exec('tmp2 = {}'.format(j))
                print(tmp2)
                comments.append(tmp2[0])
                comment_times.append(tmp2[1])
        if(comments):
            print('These comments were added {}'.format(comments))
            comments_df = pd.DataFrame(
                {'log': comments, 'trina_time': comment_times})

            comments_df['nature'] = 'comment'

            comments_df.columns = ['log', 'trina_time', 'nature']
        else:
            comments_df = pd.DataFrame(
                {'log': [], 'trina_time': [], 'nature': []})
        return comments_df
        
        

    def update_images(self):
        while(True):
            start = time.time()
            with self.images_lock:
                try:
                    tmp = deepcopy(self.jarvis.get_rgbd_images())
                    for key in tmp.keys():
                        self.images[key+'_color'].append(tmp[key][0])
                        self.images[key+'_depth'].append(tmp[key][1])
                        self.image_times[key+'_color'].append(tmp[key][2])
                        self.image_times[key+'_depth'].append(tmp[key][2])
                    del tmp
                        
                except Exception as e:
                    print(e)
                    pass

            elapsed = time.time()-start
            if(elapsed < self.images_dt):
                time.sleep(self.images_dt-elapsed)
            if(self.close_all):
                break

    def save_state_and_command_log(self):

        while(True):
            # print('saving state')
            start_time = time.time()
            commands_df = self.yield_commands_df()
            states_df = self.yield_states_df()
#             comments_df = self.yield_comments_df()
            final_df = commands_df.append(
                states_df, ignore_index=True, sort=False)
#             final_df = final_df.append(comments_df, ignore_index=True, sort=False)
            final_df.trina_time = final_df.trina_time.astype(float)
            final_df = final_df.sort_values(
                by='trina_time').drop_duplicates().reset_index(drop=True)
            final_df = final_df.astype(str)
            if(os.path.exists(self.status_filename)):
                final_df.to_csv(self.status_filename, sep='|',
                                mode='a', header=False, index=False)
            else:
                final_df.to_csv(self.status_filename, sep='|',
                                mode='a', header=True, index=False)

            elapsed = time.time()-start_time
            if(elapsed < self.intermediate_wait):
                time.sleep(self.intermediate_wait-elapsed)
            if(self.close_all):
                break

    def save_images_to_disk(self):
        while(True):
            # print('save images to disk running')
            start_time = time.time()
            with self.images_lock:
                copy_images = copy(self.images)
                copy_image_times = copy(self.image_times)
                for key in self.images.keys():
                    self.images.update({key: []})
                    self.image_times.update({key:[]})
            # print('passed images lock')
            with self.operational_lock:
                if(self.logger_items['Video']):
                    dumping_process = Process(target = dump_dataset_to_memory, args = (self.dataset_dirs,copy_images,copy_image_times))
                    dumping_process.start()
                    # print('waiting to join')
                    dumping_process.join()
                    
                    elapsed = time.time()-start_time
                    # print('\n\n\n done adding figures to disk in {} seconds'.format(elapsed))
                else:
                    print('\n\n not logging video! \n\n')
            # print('passed operational lock')
            elapsed = time.time()-start_time
            # print('elapsed = {} '.format(elapsed))
            print('\n\n\n done adding figures to disk in {} seconds'.format(elapsed))        
            if(elapsed < self.intermediate_wait):
                time.sleep(self.intermediate_wait-elapsed)
            if(self.close_all):
                break

    def safely_close(self):
        # print('safely closing hdf5 files')
        self.close_all = True
        time.sleep(5)

    def return_threads(self):
	    return [self.stateThread, self.commandThread, self.saveStateLogThread, self.updateImageThread, self.saveImageLog]
    
    def return_processes(self):
        return []
                           

        
        
        