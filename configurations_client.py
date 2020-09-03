import json
import os

class jsonReader:
    def __init__(self, name="file"):
        self.name = str(name)
        
    def read_json(self,filename):
        with open(filename, "r") as f:
            data = json.load(f)
        return data    

    def write_json(self,filename, json_object):
        try:
            print(filename)
            with open(filename, "w+") as f:
                json.dump(json_object, f)            
            return 1
        except:
            return 0    
        
    def NestedDictValues(self,d):
        for v in d.values():
            if isinstance(v, dict):
                yield from self.NestedDictValues(v)
            else:
                yield v

    def recursive_items(self,dictionary):
        for key, value in dictionary.items():
            if type(value) is dict:
                yield from self.recursive_items(value)
            else:
                yield (key, value)

    def get_entries_from_files(self,filenames):
        config_data = {}
        try:
            for name in filenames:
                with open(name) as f:
                    data = json.load(f)
                    config_data.update(data)
            return config_data 
        except Exception as e:
            print("Failed to load configurations: " + str(e))
            raise Exception

    def load_configs_with_key(self,config_filename, key):
        data = self.read_json(config_filename)
        filenames  = list(self.NestedDictValues(data[key]))
        config_data = self.get_entries_from_files(filenames)
        for key, value in self.recursive_items(config_data):
                print(key + ": " + value)
            
    def load_all_configs(self,config_filename):
        data = self.read_json("configurations.json")
        filenames  = list(self.NestedDictValues(data))
        print(filenames)
        config_data = self.get_entries_from_files(filenames)
        for key, value in self.recursive_items(config_data):
            print(key + ": " + str(value))

    def update_json_config_with_key(self,config_filename, key, value):
        json_object = self.read_json(config_filename)
        if key in json_object:
            print("Updating value of %s for %s" %(value, key))
        else:
            print("Adding new key: '%s' with value: '%s'" %(key, value))
        json_object[key] = value
        if self.write_json(config_filename,json_object):
            print("Updated")
        else:
            print("Failed to update!")

    def delete_entry(self,config_filename, key):
        json_object = self.read_json(config_filename)
        deleted_key = json_object.pop(key, None)
        if deleted_key != None and self.write_json(config_filename,json_object):
            print("Updated")
        else:
            print("Failed to update!")

    def update_json_with_dict(self,config_filename, dictionary):
        if self.write_json(config_filename,dictionary):
            print("Updated")
        else:
            print("Failed to update!")

# update_json_config("configs/ips.json", "torso", "11.1.1")


# delete the key
# each user/ip can store preferences
# add the dictionary completely
# configs -> robot -> ip -> torso
#                        -> left