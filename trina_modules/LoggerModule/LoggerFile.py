# !pip2 install pandas
import pandas as pd
import redis
from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore
import time
from threading import Thread, Lock
from copy import deepcopy, copy
import h5py
from datetime import datetime
import os
import numpy as np
import atexit


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


class StateLogger(object):
    def __init__(self, jarvis, frequency=60, image_frequency=10):
        print('\n\n\n starting logger \n\n\n')

        self.states_lock = Lock()
        self.commands_lock = Lock()
        self.images_lock = Lock()
        self.dt = 1.0/frequency
        self.images_dt = 1.0/image_frequency
        self.command_reader = TrinaQueueReader()
        self.jarvis = jarvis

        self.intermediate_wait = 20
        tmp = datetime.now()
        if(not os.path.exists('./logs')):
            os.mkdir('./logs')
        self.name = tmp.strftime("%Y_%m_%d_%H_%M_%S")
        self.jarvis.server['LOGGER_NAME'] = self.name
        self.start_all_logger_variables()
        atexit.register(self.safely_close)
        
        
        self.stateThread = Thread(target=self.update_states)
        self.commandThread = Thread(target=self.update_command_logs)
        self.saveStateLogThread = Thread(
            target=self.save_state_and_command_log)
        self.updateImageThread = Thread(target=self.update_images)
        self.saveImageLog = Thread(target=self.save_images_to_disk)
        self.stateThread.daemon = False
        self.commandThread.daemon = False
        self.saveStateLogThread.daemon = False
        self.updateImageThread.daemon = False
        self.saveImageLog.daemon = False
        self.stateThread.start()
        self.commandThread.start()
        self.updateImageThread.start()
        time.sleep(3)
        self.saveStateLogThread.start()
        self.saveImageLog.start()
    
    def start_all_logger_variables(self):
        with self.states_lock:
            with self.commands_lock:
                with self.images_lock:
                    self.status_filename = "./logs/"+self.name + ".gz"
                    self.images_filename = "./logs/"+self.name +".hdf5"
                    self.images = {}
                    self.image_times = {}
                    self.command_list = []
                    self.ui_state = []
                    self.robot_state = []
                    self.ui_times = []
                    self.robot_state_times = []
                    tmp = self.jarvis.get_rgbd_images()
                    tmp = self.jarvis.get_rgbd_images()
                    keys = tmp.keys()
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
                    self.f = h5py.File(self.images_filename, 'w')
                    for dataset_name in tmp.keys():
                        size = tmp[dataset_name][0].shape
                        self.f.create_dataset(dataset_name + '_color', (1, size[0], size[1], size[2]), chunks=True, maxshape=(
                            None, size[0], size[1], size[2]), compression='gzip', dtype='uint8')
                        self.f[dataset_name + '_color'].attrs.create('times',self.image_times[dataset_name + '_color'])

                    #then for depth
                    for dataset_name in tmp.keys():
                        size = tmp[dataset_name][1].shape
                        self.f.create_dataset(dataset_name + '_depth', (1, size[0], size[1]), chunks=True, maxshape=(
                            None, size[0], size[1]), compression='gzip')
                        self.f[dataset_name + '_depth'].attrs.create('times',self.image_times[dataset_name + '_depth'])

                    self.datasets = {}

                    for name in self.f.keys():
                        self.datasets.update({name: self.f[name]})

    def update_states(self):
        while(True):
            start = time.time()
            new_name = self.jarvis.server['LOGGER_NAME'].read()
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
        with self.states_lock:
            copy_ui_state = copy(self.ui_state)
            copy_robot_state = copy(self.robot_state)
            copy_ui_times = copy(self.ui_times)
            copy_robot_state_times = copy(self.robot_state_times)
            self.ui_state = []
            self.robot_state = []
            self.ui_times = []
            self.robot_state_times = []
        nature_ui = len(copy_ui_state)*['ui']
        nature_robot = len(copy_robot_state)*['robot']
        logs = copy_ui_state + copy_robot_state
        times = copy_ui_times + copy_robot_state_times
        natures = nature_ui + nature_robot
        states_df = pd.DataFrame(
            {'log': logs, 'trina_time': times, 'nature': natures})

        return states_df

    def yield_commands_df(self):
        with self.commands_lock:
            copy_command_list = copy(self.command_list)
            self.command_list = []

        commands = []
        command_times = []
        for i in copy_command_list:
            exec('tmp = {}'.format(i))
            for j in tmp:
                exec('tmp2 = {}'.format(j))
                commands.append(tmp2[0])
                command_times.append(tmp2[1])

        if(commands):
            commands_df = pd.DataFrame(
                {'log': commands, 'trina_time': command_times})

            commands_df['nature'] = 'command'

            commands_df.columns = ['log', 'trina_time', 'nature']
        else:
            commands_df = pd.DataFrame(
                {'log': [], 'trina_time': [], 'nature': []})
        return commands_df

    def update_images(self):
        while(True):
            start = time.time()
            with self.images_lock:
                tmp = self.jarvis.get_rgbd_images()
                for key in tmp.keys():
                    self.images[key+'_color'].append(tmp[key][0])
                    self.images[key+'_depth'].append(tmp[key][1])
                    self.image_times[key+'_color'].append(tmp[key][2])
                    self.image_times[key+'_depth'].append(tmp[key][2])
            elapsed = time.time()-start
            if(elapsed < self.images_dt):
                time.sleep(self.images_dt-elapsed)
            if(self.close_all):
                break

    def save_state_and_command_log(self):

        while(True):

            start_time = time.time()
            commands_df = self.yield_commands_df()
            states_df = self.yield_states_df()
            final_df = commands_df.append(
                states_df, ignore_index=True, sort=False)
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
            start_time = time.time()
            with self.images_lock:
                copy_images = copy(self.images)
                copy_image_times = copy(self.image_times)
                for key in self.images.keys():
                    self.images.update({key: []})
                    self.image_times.update({key:[]})
            for key in copy_images.keys():
                dset = self.datasets[key]
                dset_size = list(dset.shape)
                new_data_array = np.array(copy_images[key])
                new_dset_size = copy(dset_size)
                new_dset_size[0] += new_data_array.shape[0]
                new_dset_size = tuple(new_dset_size)
                dset.resize(new_dset_size)
                dset[dset_size[0]:dset_size[0] +
                     new_data_array.shape[0], :, :] = new_data_array
                old_times = dset.attrs.get('times')
                new_times = np.append(old_times,np.array(copy_image_times[key]))
                # print(new_dset_size)
                dset.attrs.create('times',new_times)
            self.f.flush()
            elapsed = time.time()-start_time
            if(elapsed < self.intermediate_wait):
                time.sleep(self.intermediate_wait-elapsed)
            if(self.close_all):
                break

    def safely_close(self):
        print('safely closing hdf5 files')
        self.close_all = True
        time.sleep(5)
        self.f.flush()
        self.f.close()

    def return_threads(self):
	    return [self.stateThread, self.commandThread, self.saveStateLogThread, self.updateImageThread, self.saveImageLog]
    
    def return_processes(self):
        return []