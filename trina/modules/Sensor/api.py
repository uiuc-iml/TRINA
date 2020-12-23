from trina import jarvis
from trina.utils import Promise

class SensorAPI(jarvis.APILayer):
    """External API for the sensor module."""
    def __init__(self,sensor_module,*args,**kwargs):
        self.sensor_module = sensor_module
        self.lock = sensor_module.update_lock
        jarvis.APILayer.__init__(self,*args,**kwargs)

    @classmethod
    def name(cls):
        return "sensors"

    @classmethod
    def interprocess(cls):
        return False

    def camerasAvailable(self):
        """Returns a list of strings describing available cameras"""
        with self.lock:
            return list(self.sensor_module.active_cameras.keys())

    def getRgbdImages(self,cameras=None):
        """Returns the RGBD cameras from all cameras or a subset of cameras
        corresponding to the latest images taken.

        Args:
            cameras: Can be one of the following:

                - None indicates all cameras.
                - str: indicates one camera.  (Note the return value is still a dict
                  with one key)
                - list of str: multiple cameras.

        Returns:
            dict: keys are cameras and values are (rgb,depth) image pairs.  Each
            image is a numpy object. (These are shared across threads, so be careful
            not to modify them.)
        """
        with self.lock:
            return self.sensor_module.get_rgbd_images(cameras)

    def getNextRgbdImages(self,cameras=None):
        """Returns a Promise for the next RGBD images.

        Args:
            cameras: Can be one of the following:

                - None indicates all cameras.
                - str: indicates one camera.  (Note the return value is still a dict
                  with one key)
                - list of str: multiple cameras.
        Returns:
            Promise: To get the next images when they arrive, call wait() on the
            Promise object. value() has the same format the result of
            :func:`getRgbdImages`.
        """
        with self.lock:
            p = Promise("RGBD image request from "+self._caller_name)
            if isinstance(cameras,str):
                cameras = [cameras]
            self.sensor_module.requests.append((p,'get_rgbd_images',cameras,{}))
            return p

    def getPointClouds(self,cameras=None):
        """Returns the point clouds corresponding to the latest image
        taken.

        Args:
            cameras: Can be one of the following:

                - None indicates all cameras.
                - str: indicates one camera.  (Note the return value is still a dict
                  with one key)
                - list of str: multiple cameras.

        Returns:
            dict: where keys are camera snd values are point clouds. Each
            point cloud is expressed in world coordinates as Open3D PointCloud
            objects.
        """
        with self.lock:
            return self.sensor_module.get_point_clouds(cameras)

    def getNextPointClouds(self,cameras=None):
        """Returns a Promise for the next point clouds.

        Args:
            cameras: Can be one of the following:

                - None indicates all cameras.
                - str: indicates one camera.  (Note the return value is still a dict
                  with one key)
                - list of str: multiple cameras.

        Returns:
            Promise: To get the next point clouds when they arrive, call wait() 
            on the Promise object.  value() has the same format the result of
            :func:`getPointClouds`.
        """
        with self.lock:
            p = Promise("Point cloud request from "+self._caller_name)
            if isinstance(cameras,str):
                cameras = [cameras]
            self.sensor_module.requests.append((p,'get_point_clouds',cameras,{}))
            return p

    def getLidarScan(self):
        """Returns the Lidar scan as a list of floats
        """
        with self.lock:
            return self.sensor_module.get_lidar_scan()

    def getLidarPointCloud(self):
        """Returns the Lidar scan as an N x 3 numpy array of points.
        """
        with self.lock:
            return self.sensor_module.get_lidar_point_cloud()

    def getNextLidarScan(self):
        """Returns a Promise for the Lidar scan.

        Returns:
            Promise: To get the next point clouds when they arrive, call wait() 
            on the Promise object.  value() has the same result as
            :func:`getLidarScan`.
        """
        with self.lock:
            p = Promise("Lidar scan request from "+self._caller_name)
            self.sensor_module.requests.append((p,'get_lidar_scan',None,{}))
            return p

    def getNextLidarPointCloud(self):
        """Returns a Promise for the Lidar scan.

        Returns:
            Promise: To get the next point clouds when they arrive, call wait() 
            on the Promise object.  value() has the same result as
            :func:`getLidarPointCloud`.
        """
        with self.lock:
            p = Promise("Lidar point cloud request from "+self._caller_name)
            self.sensor_module.requests.append((p,'get_lidar_point_cloud',None,{}))
            return p
