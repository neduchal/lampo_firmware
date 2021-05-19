#!/usr/bin/python3

import time
import signal
import sys
import os
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler
import logging

class Node:

    def __init__(self, name):
        self.name = name
        self.first_timestamp = time.time()
        self.timestamps = []

    def getName(self):
        return self.name

    def getTimestamp(self, topic):
        filtered = [x for x in self.timestamps if x[0] == topic]
        if len(filtered) > 0:
            return filtered[0][1]
        return self.first_timestamp

    def actualizeTimestamp(self, topic, timestamp):
        filtered = [x for x in self.timestamps if x[0] == topic]
        if len(filtered) > 0:
            filtered[0][1] = timestamp
        else:
            self.timestamps.append([topic, timestamp])


class Core:

    def __init__(self, host="localhost", port=8000, log_setting=2):
        self.server = SimpleXMLRPCServer(
            (host, port), requestHandler=SimpleXMLRPCRequestHandler, allow_none=True, logRequests=False)
        self.server.register_introspection_functions()
        self.registered_nodes = []
        self.messages = []
        self._log_setting = log_setting
        self.server.register_function(self.registerNode, "registerNode")
        self.server.register_function(self.unregisterNode, "unregisterNode")
        self.server.register_function(self.receiveMessage, "sendMessage")
        self.server.register_function(self.sendMessage, "receiveMessage")
        self.server.register_function(self.logDirectory, "logDirectory")
        self.server.register_function(self.getTopics, "getTopics")


        timestamp = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
        self.log_directory = os.path.join("log/", timestamp)
        if not os.path.exists(self.log_directory):
            os.makedirs(self.log_directory)

        logging.basicConfig(filename=os.path.join(self.log_directory, "server.log"), level=logging.DEBUG)

    def getTopics(self):
        topics = [x[1] for x in self.messages]
        print(topics)
        topics = list(set(topics))
        return topics

    def logDirectory(self):
        return self.log_directory

    def log(self, msg):
        if self._log_setting > 0:
            timestamp = time.strftime("%Y.%m.%d %H:%M:%S", time.localtime())
            if self._log_setting == 2:
                print(f"[{timestamp}]: {msg}")
            logging.info(f"[{timestamp}]: {msg}")

    def registerNode(self, name):
        for node in self.registered_nodes:
            if node.getName() == name:
                return False
        self.registered_nodes.append(Node(name))
        self.log(f"Node [{name}] registered")
        return True

    def unregisterNode(self, name):
        for i, node in enumerate(self.registered_nodes):
            if node.getName() == name:
                self.registered_nodes.pop(i)
        self.log(f"Node [{name}] unregistered")
        return True


    def receiveMessage(self, topic, msg):
        self.messages.append((time.time(), topic, msg))

    def sendMessage(self, nodename, topic):
        msgs_to_delete = [x for x, y in enumerate(
            self.messages) if y[0] < (time.time() - 5)]
        [self.messages.pop(x) for x in msgs_to_delete[::-1]]
        del msgs_to_delete
        node = [n for n in self.registered_nodes if n.getName() == nodename]
        if len(node) > 0:
            msgs_idxs = [x for x, y in enumerate(self.messages) if (
                y[1] == topic and y[0] > node[0].getTimestamp(topic))]
            if len(msgs_idxs) > 0:
                node[0].actualizeTimestamp(
                    topic, self.messages[msgs_idxs[0]][0])
                return self.messages[msgs_idxs[0]][2]
        return None

    def sigint_handler(self, sig, frame):
        self.log("Lampo Firmware Server STOPing")
        sys.exit(0)

    def run(self):
        signal.signal(signal.SIGINT, self.sigint_handler)

        print("#"*40)
        print(" LAMPO FIRMWARE SERVER v 0.1")
        print("#"*40)
        self.log("Lampo Firmware Server STARTs")
        self.server.serve_forever()


if __name__ == "__main__":
    server = Core()
    server.run()
