import json
import os

def read_json(filename):
    with open(filename, "r") as f:
        data = json.load(f)
    return data    

def write_json(filename, json_object):
    try:
        print(filename)
        with open(filename, "w+") as f:
            json.dump(json_object, f)
        
        print(json_object)
        return 1
    except:
        return 0    
    
def NestedDictValues(d):
  for v in d.values():
    if isinstance(v, dict):
      yield from NestedDictValues(v)
    else:
      yield v

def recursive_items(dictionary):
    for key, value in dictionary.items():
        if type(value) is dict:
            yield from recursive_items(value)
        else:
            yield (key, value)

def get_configs_from_files(filenames):
    config_data = {}
    try:
        for name in filenames:
            with open(name) as f:
                data = json.load(f)
                config_data.update(data)
        return config_data 
    except Exception as e:
        print("Failed to load configurations")
        raise Exception

def load_configs_with_key(config_filename, key):
   data = read_json(config_filename)
   filenames  = list(NestedDictValues(data[key]))
   config_data = get_configs_from_files(filenames)
   for key, value in recursive_items(config_data):
        print(key + ": " + value)
        
def load_all_configs(config_filename):
    data = read_json("configurations.json")
    filenames  = list(NestedDictValues(data))
    print(filenames)
    config_data = get_configs_from_files(filenames)
    for key, value in recursive_items(config_data):
        print(key + ": " + str(value))

def update_json_config(config_filename, key, value):
    json_object = read_json(config_filename)
    if key in json_object:
        print("Updating value of %s for %s" %(value, key))
    else:
        print("Adding new key: '%s' with value: '%s'" %(key, value))
    json_object[key] = value
    if write_json(config_filename,json_object):
        print("Updated")
    else:
        print("Failed to update!")

# update_json_config("configs/ips.json", "torso", "11.1.1")
data = read_json("configurations.json")
filenames  = list(NestedDictValues(data))
print(filenames)
config_data = get_configs_from_files(filenames)

# delete the key
# each user/ip can store preferences
# add the dictionary completely
# configs -> robot -> ip -> torso
#                        -> left