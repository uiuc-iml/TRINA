
class RealSenseCamera:
    def __init__(self, serial_num):
        import pyrealsense2 as rs
        self.serial_num = serial_num
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.latest_aligned_frame = None
        try:
            self.config.enable_device(serial_num.encode('utf-8'))
            self.config.enable_stream(
                rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.config.enable_stream(
                rs.stream.color, 640, 480, rs.format.rgb8, 30)
            self.align_to = rs.stream.color
            self.align = rs.align(self.align_to)
            # Start streaming
            self.pipeline.start(self.config)
            # we sleep for 3 seconds to stabilize the color image - no idea why, but if we query it soon after starting, color image is distorted.
            self.pc = rs.pointcloud()
        except Exception as e:
            print(e, 'Invalid Camera Serial Number')
            self.pipeline.stop()
        # atexit.register(self.safely_close)

    def update(self):
        frames = self.pipeline.wait_for_frames()
        if not frames.get_depth_frame() or not frames.get_color_frame():
            return
        # Fetch color and depth frames and align them
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        self.latest_aligned_frame = aligned_frames

    def latest_point_cloud(self):
        """
        Returns the point cloud from the last frame.

        Args:
        Returns:
            transformed_pc : returns the point cloud in the camera's local frame
            as a open3D PointCloud, or None if the data is not available
        """
        if self.latest_aligned_frame is None:
            print("Data Not Available at the moment")
            return None
        # Fetch color and depth frames and align them
        depth_frame = self.latest_aligned_frame.get_depth_frame()
        color_frame = self.latest_aligned_frame.get_color_frame()
        
        # Tell pointcloud object to map to this color frame
        self.pc.map_to(color_frame)
        # Generate the pointcloud and texture mappings
        points = self.pc.calculate(depth_frame)
        vtx = np.asarray(points.get_vertices())
        pure_point_cloud = np.zeros((640*480, 3))
        pure_point_cloud[:, 0] = -vtx['f0']
        pure_point_cloud[:, 1] = -vtx['f1']
        pure_point_cloud[:, 2] = -vtx['f2']
        color_t = np.asarray(color_frame.get_data()).reshape(640*480, 3)/255
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(pure_point_cloud)
        point_cloud.colors = o3d.utility.Vector3dVector(color_t)
        return point_cloud
    
    def latest_rgbd_images(self):
        if self.latest_aligned_frame is None:
            print("Data Not Available at the moment")
            return None
        depth_frame = self.latest_aligned_frame.get_depth_frame()
        color_frame = self.latest_aligned_frame.get_color_frame()
        return [color_frame.get_data(),depth_frame.get_data()]

    def safely_close(self):
        print('safely closing Realsense camera', self.serial_num)
        self.pipeline.stop()

