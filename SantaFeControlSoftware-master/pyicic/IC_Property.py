#!/usr/bin/env python
# -*- coding: utf-8 -*-

from ctypes import *

from IC_GrabberDLL import IC_GrabberDLL
from IC_Exception import IC_Exception

class IC_Property(object):

    @property
    def available(self):
        """
        """
        # returns boolean value
        iav = self._avail_funcs[self._prop_type](self._handle,
                                                 c_int(self._prop_index))
        return bool(iav)
    
    @property
    def auto_available(self):
        """
        """
        # returns boolean value
        iaa = self._auto_avail_funcs[self._prop_type](self._handle,
                                                      c_int(self._prop_index))
        return bool(iaa)

    @property
    def range(self):
        """
        Get valid range of values for the property.
        
        :returns: tuple -- (range min, range max).
        """
        rmin = c_long()
        rmax = c_long()
        err = self._range_funcs[self._prop_type](self._handle,
                                                 c_int(self._prop_index),
                                                 byref(rmin),
                                                 byref(rmax))
        if err != 1:
            raise IC_Exception(err)
        else:
            return (rmin.value, rmax.value)
        
    @property
    def min(self):
        """
        """
        return self.range[0]
    
    @property
    def max(self):
        """
        """    
        return self.range[1]
    
    @property
    def value(self):
        """
        """
        val = c_long()
        
        err = self._get_value_funcs[self._prop_type](self._handle,
                                                     c_int(self._prop_index),
                                                     byref(val))
        if err != 1:
            raise IC_Exception(err)
        else:
            return val.value
            
    @value.setter
    def value(self, val):
        """
        """
        
        # turn off auto first
        self.auto = False
        
        # set value
        err = self._set_value_funcs[self._prop_type](self._handle,
                                                     c_int(self._prop_index),
                                                     c_long(val))
        if err != 1:
            raise IC_Exception(err)
        
    @property
    def auto(self):
        """
        """
        aut = c_int()
        
        err = self._get_auto_funcs[self._prop_type](self._handle,
                                                    c_int(self._prop_index),
                                                    byref(aut))
        if err != 1:
            raise IC_Exception(err)
        else:
            return bool(aut.value)
    
    @auto.setter
    def auto(self, aut):
        """
        """
        err = self._set_auto_funcs[self._prop_type](self._handle,
                                                    c_int(self._prop_index),
                                                    c_long(int(aut)))
        if err != 1:
            raise IC_Exception(err)
    
    @property
    def type(self):
        """
        """
        return self._prop_type
    
    @staticmethod
    def get_video_property_names():
        """
        """
        return ['brightness',
                'contrast',
                'hue',
                'saturation',
                'sharpness',
                'gamma',
                'colorenable',
                'whitebalance',
                'blacklightcompensation',
                'gain']
    
    @staticmethod
    def get_camera_property_names():
        """
        """
        return ['pan',
                'tilt',
                'roll',
                'zoom',
                'exposure',
                'iris',
                'focus']
    
    @staticmethod
    def get_all_property_names():
        """
        """
        return IC_Property.get_video_property_names() + IC_Property.get_camera_property_names()
    
    def __init__(self, handle, name):
     
        self._handle = handle
        self._prop_name = name
        
        self._avail_funcs = {       'video'    :   IC_GrabberDLL.is_video_property_available,       
                                    'camera'   :   IC_GrabberDLL.is_camera_property_available}      
        self._auto_avail_funcs = {  'video'    :   IC_GrabberDLL.is_video_property_auto_available,  
                                    'camera'   :   IC_GrabberDLL.is_camera_property_auto_available} 
        self._range_funcs = {       'video'    :   IC_GrabberDLL.video_property_get_range,          
                                    'camera'   :   IC_GrabberDLL.camera_property_get_range}         
        self._get_value_funcs = {   'video'    :   IC_GrabberDLL.get_video_property,                
                                    'camera'   :   IC_GrabberDLL.get_camera_property}               
        self._set_value_funcs = {   'video'    :   IC_GrabberDLL.set_video_property,                
                                    'camera'   :   IC_GrabberDLL.set_camera_property}               
        self._get_auto_funcs = {    'video'    :   IC_GrabberDLL.get_auto_video_property,           
                                    'camera'   :   IC_GrabberDLL.get_auto_camera_property}          
        self._set_auto_funcs = {    'video'    :   IC_GrabberDLL.enable_auto_video_property,              
                                    'camera'   :   IC_GrabberDLL.enable_auto_camera_property}          
        
        vid_props = IC_Property.get_video_property_names()
        cam_props = IC_Property.get_camera_property_names()
        
        if name in vid_props:
            self._prop_type = 'video'
            self._prop_index = vid_props.index(name)
        elif name in cam_props:
            self._prop_type = 'camera'
            self._prop_index = cam_props.index(name)
        else:
            raise IC_Exception(todo)



    
    
    
    
        
