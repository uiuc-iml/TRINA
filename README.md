# TRINA

## Installing

TODO: write install instructions

TLDR; run `sudo ./trina_setup.sh` 


## On-board runtime and development

The runtime is started using the call `redrun redrun_config.json`.  By default this will run the Redis server, roscore, ros gmapping, the redrun logger, motion_server.py (using the current TRINA/Settings), trina/command_server.py, and trina/modules/UI/LocalUI.py.

The main TRINA runtime occurs on the GPU computer under trina/command_server.py.  However, the Motion module is run on the Fetch base.


## Operator UI development

Code / assets for operator UI development go under OperatorInterface/.


## Settings

In the Settings directory, you will find JSON files, which is accessed using the trina.settings accessor module.  These JSON files can refer to one another using the '@FILENAME.json' syntax, which tells the accessor to load another file to access sub-items of the root.json settings object.

Top-level keys:

- "robot": items specific to the robot model and/or hardware components.  Should refer to "@robot_CODENAME.json"
- "user": items governing operator-modifiable settings, such as preferences, saved configurations, taught policies, etc. 
- "MotionServer", "RedisServer", "CommandServer": settings used by motion_server.py / the redis server / command_server.py
- "[AppName]": settings for a specific app.


## Coding guidelines

### General

1. Treat all code in the `trina` directory as **MODULES**, not scripts.  If you have code to test the proper functioning of a module, you may put it under a `if __name__ == '__main__':` block. 
2. **More extensive testing scripts belong ** either to the `Examples/`, `Testing/`, `Hardware/`, `Diagnostics/`, or `Development/` directories. 
   - Examples is for demonstrating some functionality for training.  These should have good documentation.
   - Testing is for small scripts to evaluate functionality.  If you have arbitrary code snippets, this is where they should go
   - Hardware has a lot of stuff in it, but if you are building a specialized piece of hardware, e.g., an Arduino controller, put your code in a well-named folder here.
   - Debugging gives scripts for evaluating logs or the behavior of the robot.
   - Development is where you can put larger code bases for projects that have not yet graduated to being part of the trina module (or even trina_devel).
2. **Do not hard code anything in the trina module**.  Before committing to master, **move all hard-coded parameters to the JSON files in `Settings`**.  Access these using `trina.settings` module.
3. **Use trina_devel for experimental modules** that will eventually be added to the trina.modules folder.
4. **Name things descriptively**.  
5. **Do not commit to master just to save your work **. Add a new branch for work-in-progress (see guideline below.)
6. **Prefix new branches with your username / initials** so we can easily figure out who to contact for problems, and figure out which branches are defunct.
7. **Remove defunct code**.  If you move old code to a temp location in a different file or function that is no longer used, take it out.  If you feel like you must keep it, mark in the comments why you are keeping it around.
8. **Don't repeat yourself**.  If you need to use the same code more than once, put it into a function, class, or module.

### Robot data
1. The robot model provided by `trina.settings.robot_model_file()` (and loaded by `trina.setup.robot_model_load()`) should be always be kept up to date as the most authoritative model of the robot.  
2. To store configurations, motions, positions, and transforms, use `klampt.io.loader.save`/`load`.  You may also use `klampt.io.resource.get` / `set`.  If these are being used in Apps, store them in `Settings/MyAppName` (or better yet, `Settings/MyAppName/robot_codename`.)
3. To edit configurations, motions, positions, and transforms, use klampt.io.resource.edit.  You may also use klampt.vis to set up more sophisticated visualizations, and use vis.edit("thing_name") to edit configurations, positions, and transforms in the visualization window.


## Writing apps

Follow the template in trina/modules/apps/Example as a guideline.  

### Conventions

Modules are named via CapWords convention, without a trailing "Module".

Modules are arranged with the directory convention

- ModuleName/
    - __init__.py: imports ModuleName from module.py into the ModuleName globals
    - module.py: implements class ModuleName(jarvis.Module) which is constructed by the command server

A module will run new threads / processes upon construction.  It should stop the threads upon terminate()

### APIs

A module will access APIs using the self.jarvis object.  For example, motion commands are sent via self.jarvis.robot.

Typically available APIs:
- jarvis.robot: the Motion API (see trina/modules/Motion/api.py)
- jarvis.ui: the UI API (see trina/modules/UI/api.py)
- jarvis.sensors: the Sensors API (see trina/modules/Sensors/api.py)


### Handling multithreading
1. Be careful starting threads with class methods that access to "self". This is a recipe for disaster, because if you call a method of self or one of its objects without proper locking, you may risk calling non-reentrant code.  The `jarvis.Module` class has a `startSimpleThread` method that handles locking for you.  However, for performance, you may need to do locking yourself.
2. By default, assume that nothing is thread-safe. There are even simple things, like handles to the State Server,
   which are not reentrant.  To share data between threads, a good practice is to create a class, like `MyThreadData`, which
   contains a `threading.Lock`. The lock gets acquired every time you change the object, using code like this:

   ```
   with data.lock:
       data.foo = bar
       data.foo2[3] = bar2
   ```

   Note that Python won't have any problems with a lock-free call `data.foo = bar`, but if you all of a sudden do `data.foo[0]=bar[0]`, `data.foo[1]=bar[1]` you will run into trouble.

3. If you are creating your own threads, use `thread.daemon = True` to be Ctrl+C-friendly.

### Debugging

Modules can often be debugged by calling them as python scripts.  However, you will have to create the APIs outside of jarvis.robot and jarvis.ui yourself.  In particular, you will not have access to any shared-memory APIs, like Sensor.


### Writing API modules

APIModules are very similar to Apps, but they provide an jarvis.APILayer object for other modules to use.

They are arranged with the directory convention

- ModuleName/
    - __init__.py: imports ModuleName from module.py and ModuleNameAPI from api.py into the ModuleName globals
    - module.py: implements class ModuleName(jarvis.Module) which is constructed by the command server
    - api.py: implements class ModuleNameAPI(jarvis.APILayer) which is made accessible by other modules.

Note that an APIModule usually provides a lower-case name for the Jarvis instance provided to other modules.  For instance, Motion provides its API as "jarvis.robot", and the UI APIs provide their APIs as "jarvis.ui".

Many API communication methods are available in jarvis.APILayer, but you need to construct these using the APILayer._redisGet / _redisSet / _redisRpc / _moduleCommand methods (see comments in trina/jarvis/api.py).