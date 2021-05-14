#!/usr/bin/python3

import xmlrpc.client
import random
import time


class Node(object):

    def __init__(self, name="base_node", host="localhost", port=8000):
        self.nodename = f"{random.randint(1000000,9999999)}_{name}"
        try:
            self.client = xmlrpc.client.ServerProxy(f"http://{host}:{port}")
            self.client.registerNode(self.nodename)
        except:
            print("Server is not running")
            exit(1)
        print(self.nodename)   

    def getMessage(self, topic):
        msg = self.client.receiveMessage(self.nodename, "Test")
        if msg is not None:
            return msg

    def _handleServerMessage(self, msg):
        if msg == "exit":
            self.destroyNode()
            exit(0)

    def destroyNode(self):
        self.client.unregisterNode(self.nodename)

    def run(self):
        msg = self.getMessage("server")
        
        
        

if __name__ == "__main__":
    n = Node()
