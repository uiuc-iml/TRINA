# TRINA

## Installing

TODO: write install instructions

TLDR; run `sudo ./trina_setup.sh` 

## On-board runtime

The runtime is started using the call `redrun redrun_config.json`.  By default this will run the Redis server, roscore, ros gmapping, the redrun logger, motion_server.py (using the current TRINA/Settings), command_server.py, and UI_end_1.py.

The Command Server is 

## Settings

In the Settings directory, you will find JSON files and a `trina_settings.py` accessor module.  These JSON files can refer to one another using the '@FILENAME.json' syntax, which tells the accessor to load another file to access sub-items of the root.py settings object.

Top-level keys

- "robot": items specific to the robot model and/or hardware components.  Should refer to "@robot_CODENAME.json"
- "user": items governing operator-modifiable settings, such as preferences, saved configurations, taught policies, etc. 
- "MotionServer", "RedisServer", "CommandServer": settings used by motion_server.py / the redis server / command_server.py
- "[AppName]": settings for a specific app.

## Coding guidelines

### General

1. Treat all code in the main directory, `Jarvis/`, `Motion/`, `Settings/`, `Utils/`, and `trina_modules/` as **MODULES**, not scripts.  If you have code to test the proper functioning of the module, you may put it under a `if __name__ == '__main__':` block.  Move more sophisticated testing scripts to the `Examples/`, `Testing/`, `Hardware/`, or `Setup/` directories.
2. **Do not hard code anything in modules**.  Before committing to master, **move all hard-coded parameters to the JSON files in `Settings`**.  Access these using `trina_settings` module.
3. **Name things descriptively**.  
4. **Do not commit to master just to save your work. ** Add a new branch for work-in-progress (see guideline below.)
5. **Prefix new branches with your username / initials** so we can easily figure out who to contact for problems, and figure out which branches are defunct.
6. **Remove defunct code**.  If you move old code to a temp location in a different file or function that is no longer used, take it out.  If you feel like you must keep it, mark in the comments why you are keeping it around.
7. **Don't repeat yourself**.  If you need to use the same code more than once, put it into a function, class, or module.

### Robot data
1. The robot model provided by `trina_settings.robot_model_file()` (and loaded by `trina_settings.robot_model_load()`) should be always be kept up to date as the most authoritative model of the robot.  
2. To store configurations, motions, positions, and transforms, use `klampt.io.loader.save`/`load`.  You may also use `klampt.io.resource.get` / `set`.  If these are being used in Apps, store them in `Settings/MyAppName` (or better yet, `Settings/MyAppName/robot_codename`.)
3. To edit configurations, motions, positions, and transforms, use klampt.io.resource.edit.  You may also use klampt.vis to set up more sophisticated visualizations, and use vis.edit("thing_name") to edit configurations, positions, and transforms in the visualization window.

### Writing apps

Follow the template in trina_modules/Apps/ExampleModule as a guideline.

### Handling multithreading
1. Do not start threads with class methods that access to "self". This is a recipe for disaster, because if you call a method of self or one of its objects without proper locking, you may risk calling non-reentrant code 
2. By default, assume that nothing is reentrant. There are even simple things, like handles to the Reem server,
   which may not be reentrant.  To share data between threads is to create a class, like `MyThreadData`, which
   contains a `threading.Lock`. The lock gets acquired every time you change the object, using code like this:

   ```
   with data.lock:
       data.foo = bar
       data.foo2[3] = bar2
   ```

   Note that Python won't have any problems with a lock-free call `data.foo = bar`, but if you all of a sudden do `data.foo[0]=bar[0]`, `data.foo[1]=bar[1]` you will run into trouble.

3. Set `thread.daemon = True` to be Ctrl+C-friendly.
