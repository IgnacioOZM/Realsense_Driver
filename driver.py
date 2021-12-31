## 
# Class definition DiverRealsense
# Author: Ignacio Ortiz de Zúñiga Mingot
# Date: June 2021
# Course: 1º MII + MIC
#
# Purpose: Definition of a class to handle all functions and variables
# needed to controll the camera. It uses parameters and functions
# from the realsense SDK class pipeline.
#
# Functions:
#   - __init__():       Creates a new object of the class self, it creates a new
#                       pipe to connect with the self and sets a custom
#                       configuration.
#
#   - __del__():        Stops the pipeline before destroying the class. It has
#                       to be done to avoid errors when reconnecting.
#
#   - setCalibration(): Loads custom intrinsics and extrinsics determine
#                       by Camera_Calibration_CICLAB.py
#
#   - getColour():     Returns a colour image(1920x1080x3 uint8). It gets colour
#                       and depth data from the self, alaings it and reshape it
#                       into a matrix form.
#
#   - getDepth():      Returns a depth image(1024x768 uint16). It gets depth
#                       data from the self, alaings (if wanted) it and reshape it into a
#                       matrix form.
#
#   - getIR():         Returns a infrared image(1024x768 uint16). It gets ir
#                       data from the self, alaings (if wanted) it and reshape it into a
#                       matrix form.
#
#   - getImages():   Returns a colour image(1920x1080x3 uint8) and depth image 
#                       (1024x768 uint16) and a infrared image (1024x768 z8) if requested.
#                       It gets colour, depth and IR data from the self, alaings it and 
#                       reshape it into a matrix form.
#
#   - saveImages():     Saves a colour image(1920x1080x3 uint8) and depth image 
#                       (1024x768 uint16) and a infrared image (1024x768 z8).
#                       It gets colour, depth and IR data from the self, alaings 
#                       (if wanted) it and reshape it into a matrix form.
#
#   - transform_RGB_To_Depth(): Transform point between RGB and Depth images. It uses the
#                       camera intrinsics and extrinsics.
#
#   - getPosition():    Transform point between RGB and Depth images. It uses the
#                       camera intrinsics and extrinsics.
##

import pyrealsense2 as rs
import numpy as np
import os
from cv2 import applyColorMap, convertScaleAbs, COLORMAP_JET,  imwrite, undistort
from scipy.io import loadmat
import h5py

class realsense:

    def __init__(self) -> None:
        """Constructor. There can only be one.
        """
        # Check devices
        ctx = rs.context()
        if len(ctx.devices) > 0:
            for d in ctx.devices:
                print('Found device: ',
                    d.get_info(rs.camera_info.name), ' ',
                    d.get_info(rs.camera_info.serial_number))
                #profiles = ctx.devices[0].sensors[0].profiles
        else:
            print("No Intel Device connected")
            return
        
        # Parameters
        self._cfg = rs.config()
        self._pipe = rs.pipeline()

        # Get device product line for setting a supporting resolution
        self._pipeline_wrapper = rs.pipeline_wrapper(self._pipe)
        self._pipeline_profile = self._cfg.resolve(self._pipeline_wrapper)
        self._device = self._pipeline_profile.get_device()
        self._device_product_line = str(self._device.get_info(rs.camera_info.product_line))
        
        # Load of resolution and framerate
        self._cfg.enable_stream(rs.stream.color, 1920, 1080, rs.format.rgb8,30)
        self._cfg.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16,30)
        self._cfg.enable_stream(rs.stream.infrared, 1024, 768, rs.format.y8,30)

        # Load custom .json file
        dev = self._pipeline_profile.get_device()
        ser_dev = rs.serializable_device(dev)  
        with open("Config.json", 'r') as file:
            jsonObj = file.read().strip()       
            ser_dev.load_json(jsonObj)
            print("loaded json")
            position = jsonObj.find('"Min Distance":')
            if position != -1:
                self._min_distance = float(jsonObj[position + len('"Min Distance": '):position + len('"Min Distance": ')+4])
                self._max_distance = 9

        # Creating alingment modules
        self._align_to = rs.stream.color
        self._align = rs.align(self._align_to)
        
        # Start pipeline and wait three frames for self to auto adjust
        print("Starting conexion")
        self._pipeline_profile = self._pipe.start(self._cfg)
        for i in range(10):
            fs=self._pipe.wait_for_frames()
        
        # Set custom intrinsics and extrinsics
        self.setCalibration()

        return

    def __del__(self):
        """Constructor. There can only be one.
        It stops the pipeline with the self before closing everything.
        """
        print("Stopping conexion")
        self._pipe.stop()
        return

    def setCalibration(self):
        # Adquisition of intrinsics and extrinsics
        self._profile_colour = self._pipeline_profile.get_stream(rs.stream.color)
        self._colour_intr = self._profile_colour.as_video_stream_profile().get_intrinsics()
        
        self._profile_depth = self._pipeline_profile.get_stream(rs.stream.depth)
        self._depth_intr = self._profile_depth.as_video_stream_profile().get_intrinsics()

        self._profile_ir = self._pipeline_profile.get_stream(rs.stream.infrared)
        self._ir_intr = self._profile_ir.as_video_stream_profile().get_intrinsics()
        
        self._depth_to_colour_extrin = self._pipeline_profile.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to(self._pipeline_profile.get_stream(rs.stream.color))
        self._colour_to_depth_extrin = self._pipeline_profile.get_stream(rs.stream.color).as_video_stream_profile().get_extrinsics_to(self._pipeline_profile.get_stream(rs.stream.depth))

        # Getting the depth sensor's depth scale for positioning
        self._depth_sensor = self._pipeline_profile.get_device().first_depth_sensor()
        self._depth_scale = self._depth_sensor.get_depth_scale()
        
        if True:
            # Load of custom intrinsics & Extrinsics - cam_calibration_CICLAB
            parameters = np.load('Intrinsics&Extrinsics.npz')
            # Colour image intrinsics
            self._colour_intr.fx = parameters['mtx_Colour'][0][0]
            self._colour_intr.fy = parameters['mtx_Colour'][1][1]
            self._colour_intr.ppx = parameters['mtx_Colour'][0][2]
            self._colour_intr.ppy = parameters['mtx_Colour'][1][2]
            self._colour_intr.coeffs = parameters['dist_Colour'][0]
            # Depth image intrinsics
            self._depth_intr.fx = parameters['mtx_IR'][0][0]
            self._depth_intr.fy = parameters['mtx_IR'][1][1]
            self._depth_intr.ppx = parameters['mtx_IR'][0][2]
            self._depth_intr.ppy = parameters['mtx_IR'][1][2]
            self._depth_intr.coeffs = parameters['dist_IR'][0]
            # IR image intrinsics
            self._ir_intr.fx = parameters['mtx_IR'][0][0]
            self._ir_intr.fy = parameters['mtx_IR'][1][1]
            self._ir_intr.ppx = parameters['mtx_IR'][0][2]
            self._ir_intr.ppy = parameters['mtx_IR'][1][2]
            self._ir_intr.coeffs = parameters['dist_IR'][0]
            
            # Load of custom extrinsics
            # Colour to Depth/IR extrinsics
            R_Colour_IR = list(parameters['R_Colour_IR'][0]) + list(parameters['R_Colour_IR'][1]) + list(parameters['R_Colour_IR'][2])
            self._colour_to_depth_extrin.rotation = R_Colour_IR
            for i in range (0,3):
                self._colour_to_depth_extrin.translation[i] = parameters['T_Colour_IR'][i][0]

            # Depth/IR to Colour extrinsics
            R_IR_Colour = list(parameters['R_IR_Colour'][0]) + list(parameters['R_IR_Colour'][1]) + list(parameters['R_IR_Colour'][2])
            self._depth_to_colour_extrin.rotation = R_IR_Colour
            for i in range (0,3):
                self._depth_to_colour_extrin.translation[i] = parameters['T_IR_Colour'][i][0]


    def getColour(self, BRG: bool = False) -> np.array:
        """Function to obtain a color image.
        Args:
            -BRG: True if BRG is the desired output. By default is RGB.
        Output:
            -Image array of 1920x1080x3
        """
        # Adquisition of data
        self._frame = self._pipe.wait_for_frames()
        # Extraction of colour frame
        self._colour_frame = self._frame.get_color_frame()
        # Transformation into array
        self._colour_image = np.asanyarray(self._colour_frame.get_data())
        # Reshape for BRG format
        if BRG == True:
            self._colour_image = self._colour_image[:, :, [2, 1, 0]]

        return self._colour_image

    def getDepth(self, aling: bool = False) -> np.array:
        """Function to obtain a depth image.
        Args:
            -aling: True if both images have to be aligned. By default is False.
        Output:
            -Image array of 1020x768 -> if not aligned
        """
        # Adquisition of data
        self._frame = self._pipe.wait_for_frames()
        # Extraccion of information from frame
        if aling == True:
            self._aligned_frame = self._align.process(self._frame)
            self._depth_frame = self._aligned_frame.get_depth_frame()
        else:
            self._depth_frame = self._frame.get_depth_frame()
        # Transformation into array
        self._depth_image = np.asanyarray(self._depth_frame.get_data())

        return self._depth_image

    def getIR(self, aling: bool = False) -> np.array:
        """Function to obtain a infrared image.
        Args:
            -aling: True if both images have to be aligned. By default is False.
        Output:
            -Image array of 1020x768 -> if not aligned
        """
        # Adquisition of data
        self._frame = self._pipe.wait_for_frames()
        # Extraccion of information from frame
        if aling == True:
            self._aligned_frame = self._align.process(self._frame)
            self._ir_frame = self._aligned_frame.get_infrared_frame()
        else:
            self._ir_frame = self._frame.get_infrared_frame()
        # Transformation into array
        self._ir_image = np.asanyarray(self._ir_frame.get_data())
        return self._ir_image

    def getImages(self, BRG: bool = False, aling: bool = False, depth_colour: bool = False,
                    colour: bool = False, depth: bool = False, ir: bool = False) -> list:
        """Function to obtain all images. Colour, Depth and IR.
        Args:
            -BRG: True if BRG is the desired output. By default is RGB.
            -aling: True if both images have to be aligned. By default is False.

        Output:
            -list of array. [color, depth]
        """
        # Adquisition of data
        self._frame = self._pipe.wait_for_frames()

        # Alingment
        if aling == True:
            self._aligned_frames = self._align.process(self._frame)
            if colour:  self._colour_frame = self._aligned_frames.get_color_frame()
            if depth:   self._depth_frame = self._aligned_frames.get_depth_frame()
            if ir:      self._ir_frame = self._aligned_frames.get_infrared_frame()
        else:
            if colour:  self._colour_frame = self._frame.get_color_frame()
            if depth:   self._depth_frame = self._frame.get_depth_frame()
            if ir:      self._ir_frame = self._frame.get_infrared_frame()
        
        # Transformation into array
        if colour:  self._colour_image = np.asanyarray(self._colour_frame.get_data())
        if depth:   self._depth_image = np.asanyarray(self._depth_frame.get_data())
        if ir:      self._ir_image = np.asanyarray(self._ir_frame.get_data())

        if BRG == True:
            self._colour_image = self._colour_image[:, :, [2, 1, 0]]

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        if depth_colour == True:
            self._depth_image = applyColorMap(convertScaleAbs(self._depth_image, alpha=0.03), COLORMAP_JET)

        # Creation of return dictionary
        result = dict()
        if colour:  result['colour'] = self._colour_image
        if depth:   result['depth'] = self._depth_image
        if ir:  result['ir'] = self._ir_image
        
        return result

    def saveImages(self, path: str = "Images/", aling: bool = False, RGB: bool = True, Depth: bool = True, IR: bool = True):
        """Function to save selected images on a given path
        Args:
            -aling: True if both images have to be aligned. By default is False.
            -Colour: True if Colour image have to be saved. By default is True.
            -Depth: True if Depth image have to be saved. By default is True.
            -IR: True if IR image have to be saved. By default is True.

        Output:
            -None
        """
        # Check directory
        os.makedirs(path, exist_ok=True)
        os.makedirs(path + "Colour/", exist_ok=True)
        os.makedirs(path + "Depth/", exist_ok=True)
        os.makedirs(path + "IR/", exist_ok=True)

        # Check image counter
        pngCounter = 0
        top = os.getcwd() + "/"
        for file in os.listdir(top + path + "Colour/"):
                if file.endswith(".png"):
                    pngCounter += 1
        print("Saving images on:" + top + path)
        print("Current number of pictures: {}".format(pngCounter+1))

        # Adquisition of new and clean images with alingment
        [colour, depth, ir] = self.getAllImages(BRG = True, aling = aling, colour = False)

        # Save images
        colour_str = top + path + "Colour/" + "{:04d}".format(pngCounter) + "_Colour" + ".png"
        depth_str = top + path + "Depth/" + "{:04d}".format(pngCounter) + "_Depth" + ".png"
        ir_str = top + path + "IR/" + "{:04d}".format(pngCounter) + "_IR" + ".png"
        imwrite(colour_str, colour)
        imwrite(depth_str, depth)
        imwrite(ir_str, ir) 
        return

    def transform_RGB_To_Depth(self, RGB: list) -> list:
        """Function to transform point between RGB and Depth images.
        Args:
            -RGB(x,y): pixels of the RGB image

        Output:
            -depth_point(x,y): corresponding pixels of the Depth image
        """
        # Adquisition of not aligned image
        self._frame = self._pipe.wait_for_frames()
        self._depth_frame = self._frame.get_depth_frame()

        # Transform coordinates (x,y) to depth image
        point = [float(i) for i in RGB]
        depth_point = rs.rs2_project_color_pixel_to_depth_pixel(
            self._depth_frame.get_data(), self._depth_scale,
            self._min_distance, self._max_distance,
            self._depth_intr, self._colour_intr, self._depth_to_colour_extrin, self._colour_to_depth_extrin, point)

        if 0 <= depth_point[0] <= 1024 and 0 <= depth_point[1] <= 768:
            depth_point = (int(depth_point[0]), int(depth_point[1]))
        else:
            depth_point = (0,0)
        return depth_point

    def getPosition(self, point: list) -> list:
        """Function to obtain the global position (from camera coordinates) of
            a point in the RGB/Depth image.
        Args:
            -point(x,y): pixels of the RGB/Depth image.

        Output:
            -position of the point at the global position.
        """
        # Adquisition of not aligned image
        self._frame = self._pipe.wait_for_frames()
        self._depth_frame = self._frame.get_depth_frame()
        self._depth_image = np.asanyarray(self._depth_frame.get_data())

        # Adquisition of Z real depth
        depth_point = self.transform_RGB_To_Depth(point)
        #depth=self._depth_frame.get_distance(depth_point[0],depth_point[1])
        depth = self._depth_image[depth_point[1],depth_point[0]]*self._depth_scale  # Puede que sea necesario dar la vuelta a depth_point

        # Calculate distances   result =  [depth * x, depth * y, depth]
        position = rs.rs2_deproject_pixel_to_point(self._depth_intr, [depth_point[0],depth_point[1]], depth)
        position = [i * 100 for i in position]

        return position