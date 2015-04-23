#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 29 11:07:40 2015

@author: cdondrup
"""
#import os
#import csv
import copy
#import numpy as np
from hrsi_representation.input_multiple_abstractclass import InputMultipleAbstractclass


class OnlineInputMultiple(InputMultipleAbstractclass):
    """Reads files for training"""

    def __init__(self):
        """Creates a new instance of the FileReader class
        """
        super(OnlineInputMultiple, self).__init__()

    def generate_data_from_input(self, *args, **kwargs):
        """reads all .csv files from a given directory and returns them as
        numpy arrays

        :param path: the directory which contains the csv files

        :return: the qtc represetation of all the found files
        """
        data = kwargs["data"]

        return data
