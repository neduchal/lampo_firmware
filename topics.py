from node import Node

class Topics(Node):

     def __init__(self, i2c_address=0x3C):
        Node.__init__(self, "topics_node", "localhost", 8000, 0)
        self.client.sendMessage("test1", "message1")
        self.client.sendMessage("test1", "message2")
        self.client.sendMessage("test2", "message1")
        self.client.sendMessage("test2", "message2")
        self.client.sendMessage("test3", "message1")
        [print(x) for x in self.client.getTopics()]
        self.destroyNode()

if __name__ == "__main__":
    t = Topics()