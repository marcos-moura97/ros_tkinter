import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import tkinter.scrolledtext as ScrolledText

import threading
import logging
import time

from ros_tkinter.tk_logging import TextHandler


class ROSTkinterGUI:

    def __init__(self, master):
        self.master = master
        master.geometry("500x500")
        master.title("ROS Tkinter GUI")

        self.topics = []
        self.all_topics = {} 
        self.selected_topic = {}

        self.actual_ros_msg = {}

        self.structure()


    def structure(self):
        ## SETTING STRUCTURE
        self.label = tk.Label(self.master, text="Choose the topic that you want to listen!")
        self.label.pack()

        self.topicsCombobox = ttk.Combobox(self.master,
                                         values=self.topics)
        self.topicsCombobox.pack()
        self.topicsCombobox.bind("<<ComboboxSelected>>", self.newTopicSelected)


        # Add text widget to display logging info
        self.st = ScrolledText.ScrolledText(self.master, state='disabled')
        self.st.configure(font='TkFixedFont')
        self.st.pack()
        text_handler = TextHandler(self.st)

        # Logging configuration
        logging.basicConfig(filename='test.log',
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s')

        # Add the handler to logger
        logger = logging.getLogger()
        logger.addHandler(text_handler)

        self.stopButton = tk.Button(self.master, text="Stop", command=self.stopTopic)
        self.stopButton.pack()



    def updateTopics(self):
        self.topicsCombobox['values'] = self.topics

    #################################################
    ##              CALBACK FUNCTIONS              ##
    #################################################

    def newTopicSelected(self, event):
        if self.topicsCombobox.get() in self.all_topics:
            self.selected_topic[self.topicsCombobox.get()] = self.all_topics[self.topicsCombobox.get()]
            #print(self.selected_topic)
            msg_inicial = f'Subscribing to topic {self.topicsCombobox.get()} with message type {self.all_topics[self.topicsCombobox.get()]}'
            logging.info(msg_inicial)

    def stopTopic(self):
        if(len(self.selected_topic)==0):
            messagebox.showwarning("No topic was subscribed!", "Try to subscribe to an topic")
            return

        msg_final = f'Unsubscribing to topic...'
        logging.info(msg_final)
        self.selected_topic = {}
        self.actual_ros_msg = {}


    # test function
    def worker(self):
        # Skeleton worker function, runs in separate thread (see below)
        while True:
            # Report time / date at 2-second intervals
            time.sleep(1.0)
            if(len(self.actual_ros_msg)>0 and len(self.selected_topic)>0):
                msg = f' [{self.actual_ros_msg["_topic_name"]}]: {self.actual_ros_msg["data"]}'
                logging.info(msg)
            
            self.updateTopics()

