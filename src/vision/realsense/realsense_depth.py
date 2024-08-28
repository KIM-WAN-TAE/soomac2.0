import pyrealsense2 as rs
import numpy as np

class DepthCamera:
    def __init__(self, resolution_width, resolution_height):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        #print(img_width, img_height)
        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        depth_sensor = device.first_depth_sensor()
        # Get depth scale of the device
        self.depth_scale =  depth_sensor.get_depth_scale()
        # Create an align object
        align_to = rs.stream.color

        self.align = rs.align(align_to)
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        print("device product line:", device_product_line)
        config.enable_stream(rs.stream.depth,  resolution_width,  resolution_height, rs.format.z16, 6)
        config.enable_stream(rs.stream.color,  resolution_width,  resolution_height, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)

        profile = self.pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        self.depth_intrinsics = depth_profile.get_intrinsics()
        # print(self.get_camera_intrinsics)
        # print(self.depth_intrinsics)
       
    def get_frame(self):
    
        # Align the depth frame to color frame
        
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_image, color_image

    def get_raw_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            return False, None, None
        return True, depth_frame, color_frame
    
    def get_depth_scale(self):
        """
        "scaling factor" refers to the relation between depth map units and meters; 
        it has nothing to do with the focal length of the camera.
        Depth maps are typically stored in 16-bit unsigned integers at millimeter scale, thus to obtain Z value in meters, the depth map pixels need to be divided by 1000.
        """
        return self.depth_scale
    
    def get_camera_intrinsics(self):
        camera_params = {}

        camera_params['fx'] = self.depth_intrinsics.fx
        camera_params['fy'] = self.depth_intrinsics.fy
        camera_params['x_offset'] = self.depth_intrinsics.ppx
        camera_params['y_offset'] = self.depth_intrinsics.ppy
        camera_params['img_height'] = self.depth_intrinsics.height
        camera_params['img_width'] = self.depth_intrinsics.width


        return camera_params

    def release(self):
        self.pipeline.stop()