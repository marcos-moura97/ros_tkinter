#!/usr/bin/env python3

import asyncio
import importlib
import os
import threading
import time
import traceback

from datetime import datetime
import tkinter as tk

if os.environ.get("ROS_VERSION") == "1":
    import rospy  # ROS1
elif os.environ.get("ROS_VERSION") == "2":
    import ros_tkinter.rospy2 as rospy  # ROS2
else:
    print("ROS not detected. Please source your ROS environment\n(e.g. 'source /opt/ros/DISTRO/setup.bash')")
    exit(1)

from ros_tkinter.tkinter_interface import ROSTkinterGUI

from rosgraph_msgs.msg import Log

from ros_tkinter.serialization import ros2dict
from ros_tkinter.subscribers.dmesg_subscriber import DMesgSubscriber
from ros_tkinter.subscribers.processes_subscriber import ProcessesSubscriber
from ros_tkinter.subscribers.system_stats_subscriber import SystemStatsSubscriber
from ros_tkinter.subscribers.dummy_subscriber import DummySubscriber

class ROSTkinterNode(object):
    
    def __init__(self, node_name="ros_tkinter_node"):
        self.__class__.instance = self
        rospy.init_node(node_name)

        self.remote_subs = {}

        # actual ROS subscribers.
        # dict of topic_name -> ROS Subscriber
        self.local_subs = {}

        # minimum update interval per topic (throttle rate) amang all subscribers to a particular topic.
        # we can throw data away if it arrives faster than this
        # dict of topic_name -> float (interval in seconds)
        self.update_intervals_by_topic = {}

        # last time data arrived for a particular topic
        # dict of topic_name -> float (time in seconds)
        self.last_data_times_by_topic = {}

        if rospy.__name__ == "rospy2":
            # ros2 hack: need to subscribe to at least 1 topic
            # before dynamic subscribing will work later.
            # ros2 docs don't explain why but we need this magic.
            self.sub_rosout = rospy.Subscriber("/rosout", Log, lambda x: x) 
        
        # allows tornado to log errors to ROS
        self.logwarn = rospy.logwarn 
        self.logerr = rospy.logerr

        # mounting tkinter interface
        self.my_gui = None

        # loop to sync remote (websocket) subs with local (ROS) subs
        threading.Thread(target=self.sync_subs_loop, daemon=True).start()
        self.lock = threading.Lock()

    def start(self):
        """
        Start the ROS Node and the Tkinter GUI
        """
        root = tk.Tk()
        self.my_gui = ROSTkinterGUI(root)
        t1 = threading.Thread(target=self.my_gui.worker, args=[])
        t1.start()
        root.mainloop()
        t1.join()
        rospy.spin()


    def get_msg_class(self, msg_type):
        """
        Given a ROS message type specified as a string, e.g.
            "std_msgs/Int32"
        or
            "std_msgs/msg/Int32"
        it imports the message class into Python and returns the class, i.e. the actual std_msgs.msg.Int32

        Returns none if the type is invalid (e.g. if user hasn't bash-sourced the message package).
        """
        try:
            msg_module, dummy, msg_class_name = msg_type.replace("/", ".").rpartition(".")
        except ValueError:
            rospy.logerr("invalid type %s" % msg_type)
            return None

        try:
            if not msg_module.endswith(".msg"):
                msg_module = msg_module + ".msg"
            return getattr(importlib.import_module(msg_module), msg_class_name)
        except Exception as e:
            rospy.logerr(str(e))
            return None


    def sync_subs_loop(self):
        """
        Periodically calls self.sync_subs(). Intended to be run in a thread.
        """
        while True:
            time.sleep(1)
            self.sync_subs()

    def sync_subs(self):
        """
        Looks at self.remote_subs and makes sure local subscribers exist to match them.
        Also cleans up unused local subscribers for which there are no remote subs interested in them.
        """

        # Acquire lock since either sync_subs_loop or websocket may call this function (from different threads)
        self.lock.acquire()

        try:
            # all topics and their types as strings e.g. {"/foo": "std_msgs/String", "/bar": "std_msgs/Int32"}
            self.all_topics = {}
            topic_name_list = []
            #print(self.local_subs)
            for topic_tuple in rospy.get_published_topics():
                topic_name = topic_tuple[0]
                topic_type = topic_tuple[1]
                topic_name_list.append(topic_name)
                if type(topic_type) is list:
                    topic_type = topic_type[0]  # ROS2
                self.all_topics[topic_name] = topic_type
                
            self.my_gui.topics = topic_name_list
            self.my_gui.all_topics = self.all_topics

            self.remote_subs = self.my_gui.selected_topic
            for topic_name in self.remote_subs:
                #if len(self.remote_subs[topic_name]) == 0:
                #    continue

                # remote sub special (non-ros) topic: _dmesg
                # handle it separately here
                if topic_name == "_dmesg":
                    if topic_name not in self.local_subs:
                        rospy.loginfo("Subscribing to dmesg [non-ros]")
                        self.local_subs[topic_name] = DMesgSubscriber(self.on_dmesg)
                    continue

                if topic_name == "_system_stats":
                    if topic_name not in self.local_subs:
                        rospy.loginfo("Subscribing to _system_stats [non-ros]")
                        self.local_subs[topic_name] = SystemStatsSubscriber(self.on_system_stats)
                    continue

                if topic_name == "_top":
                    if topic_name not in self.local_subs:
                        rospy.loginfo("Subscribing to _top [non-ros]")
                        self.local_subs[topic_name] = ProcessesSubscriber(self.on_top)
                    continue

                # check if remote sub request is not actually a ROS topic before proceeding
                if topic_name not in self.all_topics:
                    rospy.logwarn("warning: topic %s not found" % topic_name)
                    continue

                # if the local subscriber doesn't exist for the remote sub, create it
                if topic_name not in self.local_subs:
                    topic_type = self.all_topics[topic_name]
                    msg_class = self.get_msg_class(topic_type)

                    if msg_class is None:
                        # invalid message type or custom message package not source-bashed
                        # put a dummy subscriber in to avoid returning to this again.
                        # user needs to re-run rosboard with the custom message files sourced.
                        self.local_subs[topic_name] = DummySubscriber()
                        
                        continue

                    self.last_data_times_by_topic[topic_name] = 0.0

                    rospy.loginfo("Subscribing to %s" % topic_name)

                    self.local_subs[topic_name] = rospy.Subscriber(
                        topic_name,
                        self.get_msg_class(topic_type),
                        self.on_ros_msg,
                        callback_args=(topic_name, topic_type),
                    )

            # clean up local subscribers for which remote clients have lost interest
            for topic_name in list(self.local_subs.keys()):
                if topic_name not in self.remote_subs or len(self.remote_subs[topic_name]) == 0:
                    rospy.loginfo("Unsubscribing from %s" % topic_name)
                    #                        if(topic_name == '/rosout'):
                    # self.log_file()
                    self.local_subs[topic_name].unregister()
                    del self.local_subs[topic_name]

        except Exception as e:
            rospy.logwarn(str(e))
            traceback.print_exc()

        self.lock.release()

    def on_ros_msg(self, msg, topic_info):
        """
        ROS messaged received (any topic or type).
        """
        topic_name, topic_type = topic_info
        t = time.time()
        
        #if t - self.last_data_times_by_topic.get(topic_name, 0) < self.update_intervals_by_topic[topic_name] - 1e-4:
        #    return
        
        # convert ROS message into a dict and get it ready for serialization
        ros_msg_dict = ros2dict(msg)
        
        # add metadata
        ros_msg_dict["_topic_name"] = topic_name
        ros_msg_dict["_topic_type"] = topic_type
        ros_msg_dict["_time"] = time.time() * 1000

        # log last time we received data on this topic
        self.last_data_times_by_topic[topic_name] = t
        #print(ros_msg_dict)
        self.my_gui.actual_ros_msg = ros_msg_dict


        
def main():
    ROSTkinterNode().start()

if __name__ == '__main__':
    main()
