#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 29 11:36:24 2015

@author: cdondrup
"""

import rospy
import numpy as np
from qsrlib_io.world_trace import Object_State, World_Trace
from qsrlib_ros.qsrlib_ros_client import QSRlib_ROS_Client
from qsrlib.qsrlib import QSRlib_Request_Message
from hrsi_representation.input_base_abstractclass import InputBaseAbstractclass
from abc import abstractmethod, ABCMeta
try:
    import cPickle as pickle
except:
    import pickle


class InputMultipleAbstractclass(InputBaseAbstractclass):
    """Provides functions for:
        - the transformation of raw data into qsr_lib format
        - converting the data into QTC using qsr_lig

    Will be used as a base class for the training and online data input classes
    """

    __metaclass__ = ABCMeta

    def __init__(self):
        """Creates a new instance of the InputBaseClass"""
        super(InputMultipleAbstractclass, self).__init__()
#        self.qtc_types = {
#            "qtcb": "qtc_b_simplified",
#            "qtcc": "qtc_c_simplified",
#            "qtcbc": "qtc_bc_simplified"
#        }
#        self.template = {
#            "agent1": {
#                "name": "",
#                "x": np.array([]),
#                "y": np.array([])
#            },
#            "agent2": {
#                "name": "",
#                "x": np.array([]),
#                "y": np.array([])
#            }
#        }
#        self.qtc = None

    def _convert_to_world(self, data_dict, quantisation_factor=0, validate=True, no_collapse=False, distance_threshold=np.Inf):
        world = World_Trace()

        ob = []
        for elem in data_dict:
            for idx, cord in enumerate(elem[elem.keys()[0]]["data"]): 
                ob.append(Object_State(
                    name=elem.keys()[0],
                    timestamp=idx,
                    x=cord[0],
                    y=cord[1],
                    quantisation_factor=quantisation_factor,
                    validate=validate,
                    no_collapse=no_collapse,
                    distance_threshold=distance_threshold
                ))

        world.add_object_state_series(ob)
        return world

    @abstractmethod
    def generate_data_from_input(self, *args, **kwargs):
        """Input data into the conversion process"""
        pass

    def convert(self, data, qtc_type, quantisation_factor=0, validate=True, no_collapse=False, distance_threshold=np.Inf):
        """Convert data inserted via put() into QTC

        :param qtc_type: qtcb|qtcc|qtcbc
        """
        data = [data] if not isinstance(data, list) else data
        ret = []
        world = self._convert_to_world(
            data_dict=data,
            quantisation_factor=quantisation_factor,
            validate=validate,
            no_collapse=no_collapse,
            distance_threshold=distance_threshold
        )

        try:
            qsr = self.qtc_types[qtc_type]
        except KeyError:
            rospy.logfatal("Unknown QTC type: %s" % qtc_type)
            return
        ret, ob = self._request_qtc(qsr=qsr, world=world)
        return ret, ob
#
#    def _to_np_array(self, string):
#        return np.fromstring(string.replace('-','-1').replace('+','+1'), dtype=int, sep=',')
