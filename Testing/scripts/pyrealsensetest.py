import pyrealsense2 as rs
pipeline = rs.pipeline()
config = rs.config()

config.enable_device('f0233155'.encode('utf-8'))

# config.enable_device(serial_num.encode('utf-8'))
config.enable_stream(
rs.stream.depth,1024,768, rs.format.z16, 30)
config.enable_stream(
rs.stream.color, 1024, 768, rs.format.rgb8, 30)
# print(help(config))
align_to = rs.stream.color
align = rs.align( align_to)
# Start streaming
pipeline.start( config)
# we sleep for 3 seconds to stabilize the color image - no idea why, but if we query it soon after starting, color image is distorted.
pc = rs.pointcloud()