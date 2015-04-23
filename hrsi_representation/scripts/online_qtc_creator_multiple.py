#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Feb 27 16:03:38 2015

@author: cdondrup
"""

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from bayes_people_tracker.msg import PeopleTracker
from dynamic_reconfigure.server import Server as DynServer
from hrsi_representation.cfg import OnlineQTCCreatorConfig
from hrsi_representation.msg import QTCArray, QTC
from hrsi_representation.online_input_multiple import OnlineInputMultiple
import numpy as np
import tf
import json

class OnlineQTCCreator(object):
    """Creates QTC state sequences from online input"""

    _qtc_types = {
        0: "qtcb",
        1: "qtcc",
        2: "qtcbc"
    }
    _robot_pose = None
    _buffer = dict()

    def __init__(self, name):
        rospy.loginfo("Starting %s" % name)
        self.input        = OnlineInputMultiple()
        ppl_topic         = rospy.get_param("~ppl_topic", "/people_tracker/positions")
        self.target_frame = rospy.get_param("~target_frame", "/map")
        self.dyn_srv      = DynServer(OnlineQTCCreatorConfig, self.dyn_callback)
        self.listener     = tf.TransformListener()
        self.pub          = rospy.Publisher("~qtc_array", QTCArray, queue_size=10)
        rospy.Subscriber(
            ppl_topic,
            PeopleTracker,
            callback=self.ppl_callback,
            queue_size=1
        )

    def dyn_callback(self, config, level):
        self.qtc_type            = self._qtc_types[config["qtc_type"]]
        self.quantisation_factor = config["quantisation_factor"]
        self.distance_threshold  = config["distance_threshold"]
        self.validate            = config["validate"]
        self.no_collapse         = config["no_collapse"]
        self.smoothing_rate      = config["smoothing_rate"]
        return config

    def ppl_callback(self, msg):
        out = QTCArray()
        out.header = msg.header
        out.header.frame_id = self.target_frame

        for (uuid, pose) in zip(msg.uuids, msg.poses):
            person = PoseStamped()
            person.header = msg.header
            person.pose = pose
            try:
                self.listener.waitForTransform(msg.header.frame_id, self.target_frame, msg.header.stamp, rospy.Duration(1.0))
                transformed = self.listener.transformPose(self.target_frame, person)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
                rospy.logwarn(ex)
                return

            if not uuid in self._buffer.keys():
                self._buffer[uuid] = {"data": np.array(
                    [
                        transformed.pose.position.x,
                        transformed.pose.position.y
                    ]
                ).reshape(-1,2), "last_seen": msg.header.stamp.to_sec()}
            else:
                self._buffer[uuid]["data"] = np.append(
                    self._buffer[uuid]["data"],
                    [
                        transformed.pose.position.x,
                        transformed.pose.position.y
                    ]
                ).reshape(-1,2)
#                if self._buffer[uuid]["data"].shape[0] > 3:
#                    self._buffer[uuid]["data"] = self._buffer[uuid]["data"][:-1] if np.allclose(self._buffer[uuid]["data"][-1], self._buffer[uuid]["data"][-3]) else self._buffer[uuid]["data"]
            self._buffer[uuid]["last_seen"] = msg.header.stamp.to_sec()

        data = []        
        for uuid, elem in self._buffer.iteritems():
            if elem["data"].shape[0] > 2:
                data.append({uuid: elem})
                
        qtc, between = self.input.convert(
            data=self.input.generate_data_from_input(
                data=data
            ),
            qtc_type=self.qtc_type,
            quantisation_factor=self.quantisation_factor,
            validate=self.validate,
            no_collapse=self.no_collapse,
            distance_threshold=self.distance_threshold
        )
        
        #print between        
        #print qtc
        res = []
        for i, b in enumerate(between):
            res.append([])
            idx = i            
            while idx < len(qtc):
          #      print i, idx
                res[i].append(qtc[idx])
                idx = idx + len(between)
         #       print idx
                
        print res[0]
        print res[1]
            

#        qtc_msg                     = QTC()
#        qtc_msg.collapsed           = not self.no_collapse
#        qtc_msg.qtc_type            = self.qtc_type
#        qtc_msg.k                   = "Robot"
#        qtc_msg.l                   = "Human"
#        qtc_msg.quantisation_factor = self.quantisation_factor
#        qtc_msg.distance_threshold  = self.distance_threshold
#        qtc_msg.smoothing_rate      = self.smoothing_rate
#        qtc_msg.validated           = self.validate
#        qtc_msg.uuid                = uuid
#        qtc_msg.qtc_serialised      = json.dumps(qtc.tolist())
#
#        out.qtc.append(qtc_msg)
#
#        self.pub.publish(out)
        self.decay()
        rospy.sleep(self.smoothing_rate)

    def decay(self):
        for uuid in self._buffer.keys():
            if self._buffer[uuid]["last_seen"] + 120. < rospy.Time.now().to_sec():
                del self._buffer[uuid]


if __name__ == "__main__":
    rospy.init_node("online_qtc_creator_multiple")
    oqc = OnlineQTCCreator(rospy.get_name())
    rospy.spin()
