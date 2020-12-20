import json
import copy
import os

class jsonReader:
    def __init__(self, name = "configs", filename="meta.json"):
        self.name = name
        self.base_config_file = filename
        self.load_all_configs()
        
    def read_json(self,filename):
        if not filename.endswith("json"):
            raise Exception("Can't write this file format. Use JSON format")
        
        with open(filename, "r") as f:
            data = json.load(f)
        return data    

    def write_json(self,filename, json_object):
        if not filename.endswith("json"):
            raise Exception("Can't write this file format. Use JSON format")

        try:
            f =  open(filename, "w+")
            json.dump(json_object, f, indent=4)            
            return 1
        except:
            raise Exception("Failed to write to file")
        
    def flattenNestedDictValues(self,d):
        for k,v in d.items():
            if isinstance(v, dict):
                yield from self.flattenNestedDictValues(v)
            else:
                yield (k,v)

    def recursive_items(self,dictionary):
        for key, value in dictionary.items():
            if type(value) is dict:
                yield from self.recursive_items(value)
            else:
                yield (key, value)

    def get_entries_from_files(self,filenames):
        config_data = {}
        for name in filenames:
            config_data.update(self.get_entry(name))
        return config_data

    def get_entry(self, filename):
        try:
            return self.read_json(filename)
        except Exception as e:
            print("Failed to load configurations: " + str(e))
            raise Exception
    
    def write_entry(self, filedict, datadict, isEmpty=False):
        for k, v in filedict.items():
            if isinstance(v, dict):
                self.write_entry(filedict[k],datadict[k], isEmpty)
            else:
                if isEmpty:
                    self.write_json(v,{})
                elif datadict[k] == self.data[k]:
                    if self.write_json(v,datadict[k]):
                        self.load_all_configs
                    else:
                        raise Exception("Update Failed")
        return 1

    def update_file_config_with_key(self,config_filename, key, value):
        json_object = self.read_json(config_filename)
        if key in json_object:
            print("Updating key: '%s' with value: %s" %(key, value))
        else:
            print("Adding new key: '%s' with value: %s" %(key, value))
        json_object[key] = value
        if self.write_json(config_filename,json_object):
            self.load_all_configs()
            print("Updated")
        else:
            raise Exception("Update Failed")

    def delete_entry(self,config_filename, key):
        json_object = self.read_json(config_filename)
        deleted_key = json_object.pop(key, None)
        if deleted_key != None and self.write_json(config_filename,json_object):
            self.load_all_configs()
            print("Deleted "+ key + "from "+ config_filename )
        else:
            raise Exception("Deletion Failed")

    def update_file_with_dict(self,config_filename, dictionary):
        if self.write_json(config_filename,dictionary):
            self.load_all_configs()
            print("Updated")
        else:
            print("Failed to update!")

    def update_directory(self, new_dir):
        self.directory.update(new_dir)
        if self.write_json(self.base_config_file,self.directory):
            self.load_all_configs()
            print("Updated")
        else:
            print("Failed to update!")

    def load_configs(self, file_dict):
        for k, v in file_dict.items():
            if isinstance(v, dict):
                file_dict[k] = self.load_configs(v)
            else:
                file_dict[k] = self.get_entry(v)
        return file_dict

    def load_all_configs(self):
        self.directory = self.read_json(self.base_config_file)
        files = self.read_json(self.base_config_file)
        self.data = self.load_configs(files)

    def add_new_file(self, file_dict, data_dict, isFirst=True):
        print(file_dict, isFirst)
        for k, v in file_dict.items():
            if isinstance(v, dict):
                data_dict[k] = v
                self.add_new_file(file_dict[k],data_dict[k], False)
            else:
                # data_dict[k] = {}
                1==1
        if isFirst == True:
            print(file_dict)
            self.data.update(data_dict)
            self.directory.update(file_dict)
            if self.write_json(self.base_config_file,self.directory):
                print("Updated base configs")
            self.write_entry(file_dict,self.data, True)

    def get_directory(self):
        return copy.deepcopy(self.directory)

    def get_data(self):
        return copy.deepcopy(self.data)

    def get_name(self):
        return self.name

    def get_base_config_name(self):
        return self.base_config_file

    def get_values_list(self):
        return list(self.flattenNestedDictValues(self.data))
