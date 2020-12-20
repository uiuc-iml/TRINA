class ZedCamera:
    def __init__(self, serial_num):
        # only import pyzed if running on python3
        if(sys.version_info[0] < 3):
            pass
        else:
            import pyzed.sl as sl
        # Note: This code presumes the zed cameras are fixed w.r.t. the base.
        # Create a Camera object
        self.zed = sl.Camera()
        self.serial_num = serial_num
        self.connected = False
        
        self.point_cloud = sl.Mat()
        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.sdk_verbose = False
        # init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_resolution = sl.RESOLUTION.HD2K
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        init_params.set_from_serial_number(serial_num)
        init_params.depth_minimum_distance = 0.20
        init_params.depth_maximum_distance = 40
        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.runtime_parameters = sl.RuntimeParameters()
        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print('There was an error while trying to access the zed camera, please review and try again.')
    
    def update(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.connected = True
        else:
            self.connected = False

    def latest_point_cloud(self):
        if not self.connected:
            return None
        self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
        pc = self.point_cloud.get_data()

        float_color = pc[:, :, 3]
        final_shape = float_color.shape
        data = float_color.flatten(order='F').view(np.uint8)
        red = data[::4].reshape(final_shape, order='F')
        green = data[1::4].reshape(final_shape, order='F')
        blue = data[2::4].reshape(final_shape, order='F')
        alpha = data[3::4].reshape(final_shape, order='F')

        color_pic = np.zeros(
            (final_shape[0], final_shape[1], 3), dtype=np.uint8)
        color_pic[:, :, 0] = red
        color_pic[:, :, 1] = green
        color_pic[:, :, 2] = blue
        color_t = np.asarray(color_pic).reshape(-1, 3)/255

        # pc = np.nan_to_num(pc)
        reshaped_pc = np.zeros((pc.shape[0]*pc.shape[1], 3))
        reshaped_pc[:, 0] = pc[:, :, 0].flatten()
        reshaped_pc[:, 1] = pc[:, :, 1].flatten()
        reshaped_pc[:, 2] = pc[:, :, 2].flatten()
        reshaped_pc = np.nan_to_num(reshaped_pc, 0, posinf=0, neginf=0)
        # we must now process color - and here comes a lot of bit shifting

        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(reshaped_pc)
        point_cloud.colors = o3d.utility.Vector3dVector(color_t)
        # we then finally transform the point cloud to the robot's coordinates:
        return point_cloud

    def latest_rgbd_images(self):
        if not self.connected:
            print("Data Not Available at the moment")
            return None
        self.zed.retrieve_image(self.image, sl.VIEW.LEFT) # Get the left image
        self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH) # Retrieve depth Mat. Depth is aligned on the left image
        return([self.image.get_data(),self.depth.get_data()])
    
    def safely_close(self):
        print('safely closing zed camera ', self.serial_num)
        self.zed.close()
