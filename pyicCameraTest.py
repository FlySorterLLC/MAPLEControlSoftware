import pyicic.IC_ImagingControl
import time
import numpy as np
import cv2

# open lib
ic_ic = pyicic.IC_ImagingControl.IC_ImagingControl()
ic_ic.init_library()

# open first available camera device
cam_names = ic_ic.get_unique_device_names()
cam = ic_ic.get_device(cam_names[0])
cam.open()

#cam_properties = cam.list_property_names()
#print "Available camera properties:", cam_properties

#cam.saturation.value = 100
#cam.hue.value=0
cam.gain.value = 10
cam.exposure.value = -7

#print cam.list_video_formats()
#cam.set_video_format('BY8 (1920x1080)')
#cam.set_video_format('BY8 (640x480)')
#cam.set_video_format('BY8 (2592x1944)')
#cam.set_frame_rate(4.00)

cam.prepare_live()
cam.start_live()
cam.snap_image()
(imgdata, w, h, d) = cam.get_image_data()
cam.stop_live()

print "Image is", w, "by", h

img = np.ndarray(buffer = imgdata,
                 dtype = np.uint8,
                 shape = (h, w, d))

# cv2.imwrite("camTest.bmp", img)

cam.close()
ic_ic.close_library()
