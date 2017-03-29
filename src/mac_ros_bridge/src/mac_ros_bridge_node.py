#!/usr/bin/env python2

# Thanks to Python-DTU for inspiration!
# Parts of code taken from Communicator class

import roslib
import rospy

#import geometry_msgs.msg

import socket
import threading
import time

import xml.etree.cElementTree as eT
from symbol import except_clause
from std_msgs.msg import String


ADDRESS = ('130.149.232.33', 12300)
RETRY_DELAY = 1.0
RECV_SIZE = 8192

PASSWORD = "1"

SEPARATOR = b'\0'

class MacRosBridge (threading.Thread):
    def __init__(self, name):
        threading.Thread.__init__(self)
        print "MacRosBridge::init"

        rospy.init_node('mac_ros_bridge_node', anonymous=True)

        self.agent_name = rospy.get_param('~agent_name', 'UNKNOWN')
        print "nodename =", rospy.get_name() #name is with leading slash / namespace

#        self.name = name
        self.auth = eT.fromstring('''<?xml version="1.0" encoding="UTF-8" standalone="no"?>
            <message type="auth-request"><authentication password="test" username="test"/></message>''')
        self.message_id = -1
        
        rospy.Subscriber("action", String, self.callback)
        
        self.pub = rospy.Publisher('~perception', String, queue_size = 10)
        
        self.rate = rospy.Rate(10) # 10hz
        
    def connect(self): # -> bool:
        try:
            print "Connecting...", self.agent_name
            self.s = socket.create_connection(ADDRESS, RETRY_DELAY)
            self.s.settimeout(None) # enable blocking mode until simulation starts
            return True
        except OSError as error:
            print('Error connecting to {}: {}'.format(ADDRESS, error))
            return False
        except socket.timeout:
            print('Error connecting to {}: {}'.format(ADDRESS, "timeout"))
            return False
        except socket.error as e:
            print('Error connecting to {}: {}'.format(ADDRESS, e))
            return False
        
    def reconnect(self):
        try:
            self.s.shutdown(socket.SHUT_RDWR)
            self.s.close()
        except OSError:
            pass
        time.sleep(RETRY_DELAY)
        print('Reconnecting...')
        self.s = None
        while not self.connect():
            time.sleep(RETRY_DELAY)
        self.authenticate()
        
    def authenticate(self):
        auth_element = self.auth.find('authentication')
        auth_element.attrib['username'] = self.agent_name
        auth_element.attrib['password'] = PASSWORD
        self.s.send(eT.tostring(self.auth) + SEPARATOR)
        
    def handle_message(self, xml):
        message = eT.fromstring(xml)
        typ = message.get('type')

        if typ == 'request-action':
            self.id = message.find('perception').get('id')
            print "request-action: perception id = ", self.id
            self.message_id = message.find('perception').get('id')
            
            # TODO doing something
            
#            self.send("skip")
            self.publish_perception("" + self.message_id)
            
        elif typ == 'sim-start':
            print "sim-start: steps = ", message.find('simulation').get('steps')
        elif typ == 'auth-response':
            print "auth-response", message.find('authentication').get('result')
        elif typ == 'sim-end':
            print "ranking=", message.find('sim-result').get('ranking')
            print "score=", message.find('sim-result').get('score')
        elif typ == 'bye':
            print "Do somethingmeaningful on system exit"
            
    def send(self, message):
            response = eT.Element('message', type='action')
            action = eT.SubElement(response, 'action', id=self.message_id, type=message)
            self.s.send(eT.tostring(response) + SEPARATOR)
        

    def run(self):
        print "MacRosBridge::run"
        while not self.connect():
            time.sleep(RETRY_DELAY)
        self.authenticate()
        buffer = b''
        while True:
            try:
                data = self.s.recv(RECV_SIZE)
            except socket.timeout:
                print "socket timeout"
                self.reconnect()
                buffer = b''
                continue
            except socket.error as e:
                if e.errno == errno.ECONNRESET:
                    # connection closed by server
                    data = b''
                else:
                    print('Socket error: {}'.format(e))
                    self.reconnect()
                    buffer = b''
                    continue
            if len(data) == 0:
                print('Connection closed by server')
                self.reconnect()
                buffer = b''
                continue
            else:
                buffer += data
                index = buffer.find(SEPARATOR)
                while index != -1:
                    self.handle_message(buffer[0:index].decode())
                    buffer = buffer[index + 1:]
                    index = buffer.find(SEPARATOR)

    def callback(self, data):
        print "MacRosBridge::callback", data.data
        self.send(data.data)
    
    def publish_perception(self, percept_str):
        print "MacRosBridge::publish_perception", percept_str
        rospy.loginfo(percept_str)
        self.pub.publish(percept_str)
#        self.rate.sleep()
        
if __name__ == '__main__':
    print "mac_ros_bridge_node::main"
    try:
        bridge = MacRosBridge(rospy.get_param('~agent_name', 'UNKNOWN')).start()
#     bridge = MacRosBridge("a2").start()
#     bridge = MacRosBridge("a3").start()
#     bridge = MacRosBridge("a4").start()
#     time.sleep(RETRY_DELAY)
#     bridge = MacRosBridge("a5").start()
#     bridge = MacRosBridge("a6").start()
#     bridge = MacRosBridge("a7").start()
#     bridge = MacRosBridge("a8").start()
#     time.sleep(RETRY_DELAY)
#     bridge = MacRosBridge("a9").start()
#     bridge = MacRosBridge("a10").start()
#     bridge = MacRosBridge("a11").start()
#     bridge = MacRosBridge("a12").start()
#     time.sleep(RETRY_DELAY)
#     bridge = MacRosBridge("a13").start()
#     bridge = MacRosBridge("a14").start()
#     bridge = MacRosBridge("a15").start()
#     bridge = MacRosBridge("a16").start()
#     time.sleep(RETRY_DELAY)
# 
#     bridge = MacRosBridge("b1").start()
#     bridge = MacRosBridge("b2").start()
#     bridge = MacRosBridge("b3").start()
#     bridge = MacRosBridge("b4").start()
#     time.sleep(RETRY_DELAY)
#     bridge = MacRosBridge("b5").start()
#     bridge = MacRosBridge("b6").start()
#     bridge = MacRosBridge("b7").start()
#     bridge = MacRosBridge("b8").start()
#     time.sleep(RETRY_DELAY)
#     bridge = MacRosBridge("b9").start()
#     bridge = MacRosBridge("b10").start()
#     bridge = MacRosBridge("b11").start()
#     bridge = MacRosBridge("b12").start()
#     time.sleep(RETRY_DELAY)
#     bridge = MacRosBridge("b13").start()
#     bridge = MacRosBridge("b14").start()
#     bridge = MacRosBridge("b15").start()
#     bridge = MacRosBridge("b16").start()
#     time.sleep(RETRY_DELAY)
        print "before spin()"
        rospy.spin()
        print "after spin()"

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
