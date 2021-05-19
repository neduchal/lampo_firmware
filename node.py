#!/usr/bin/python3

import xmlrpc.client
import random
import time
import signal
import sys
import logging
import os

class Node(object):

    def __init__(self, name="base_node", host="localhost", port=8000, log_setting = 2):
        self.nodename = f"{random.randint(1000000,9999999)}_{name}"
        self._log_setting = log_setting
        self.log_directory = None
        try:
            self.client = xmlrpc.client.ServerProxy(f"http://{host}:{port}")
            self.client.registerNode(self.nodename)
            self.log_directory = self.client.logDirectory()
        except:
            print("Server is not running")
            exit(1)
        if self._log_setting > 0:
            logging.basicConfig(filename=os.path.join(self.log_directory, f"{self.nodename}.log"), level=logging.DEBUG)
            self.log("Node started")

    def log(self, msg):
        if self._log_setting > 0:
            timestamp = time.strftime("%Y.%m.%d %H:%M:%S", time.localtime())
            if self._log_setting == 2:
                print(f"[{timestamp}]: {msg}")
            logging.info(f"[{timestamp}]: {msg}")

    def getMessage(self, topic):
        try:
            msg = self.client.receiveMessage(self.nodename, topic)
            if msg is not None:
                return msg
        except:
            print("Lampo Server is not running")
            sys.exit(0)


    def _handleServerMessage(self):
        msg = self.getMessage("server")
        pass

    def destroyNode(self):
        print(self.client.unregisterNode(self.nodename))

    def sigint_handler(self, sig, frame):
        print("Node STOPing")
        self.destroyNode()
        sys.exit(0)        

    def main(self):
        pass

    def run(self):
        signal.signal(signal.SIGINT, self.sigint_handler)
        while True:           
            #self._handleServerMessage()
            self.main()

        
        
        

if __name__ == "__main__":
    n = Node()
    n.run()
