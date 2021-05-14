#!/usr/bin/python3

import time
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler

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
        print(self.timestamps)
        filtered = [x for x in self.timestamps if x[0] == topic]
        print(filtered)
        if len(filtered) > 0:
            filtered[0][1] = timestamp
        else:
            self.timestamps.append([topic, timestamp])
        print(self.timestamps)

class Core:

    def __init__(self, host="localhost", port=8000):
        self.server = SimpleXMLRPCServer((host, port), requestHandler=SimpleXMLRPCRequestHandler, allow_none=True)
        self.server.register_introspection_functions()
        self.registered_nodes = []
        self.messages = []

        self.server.register_function(self.registerNode, "registerNode")
        self.server.register_function(self.unregisterNode, "unregisterNode")
        self.server.register_function(self.receiveMessage, "sendMessage")
        self.server.register_function(self.sendMessage, "receiveMessage")

    def registerNode(self, name):
        for node in self.registered_nodes:
            if node.getName() == name:
                return False                       
        self.registered_nodes.append(Node(name))
        print(f"Node {name} added")
        return True

    def unregisterNode(self, name):
        for i, node in enumerate(self.registered_nodes):
            if node.getName() == name:
                self.registered_nodes.pop(i)
        

    def receiveMessage(self, topic, msg):
        self.messages.append((time.time(), topic, msg))
        print(f"Recieved message {msg} to topic {topic}.")

    def sendMessage(self, nodename, topic):     
        msgs_to_delete = [x for x, y in enumerate(self.messages) if y[0] < (time.time() - 5)]
        print(len(self.messages), msgs_to_delete)
        [self.messages.pop(x) for x in msgs_to_delete[::-1]]
        del msgs_to_delete 
        node = [n for n in self.registered_nodes if n.getName() == nodename]
        if len(node) > 0:
            msgs_idxs = [x for x, y in enumerate(self.messages) if (y[1] == topic and  y[0] > node[0].getTimestamp(topic))]
            if len(msgs_idxs) > 0:
                node[0].actualizeTimestamp(topic, self.messages[msgs_idxs[0]][0])
                return self.messages[msgs_idxs[0]][2]
        return None

    def run(self):
        self.server.serve_forever()


if __name__=="__main__":
    server = Core()
    server.run()
        
        

    
