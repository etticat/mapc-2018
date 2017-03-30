#!/usr/bin/env python2

# Thanks to Python-DTU for inspiration!
# Parts of code taken from Communicator class

import roslib
import rospy

from mac_ros_bridge.msg import RequestAction, GenericAction

import socket
import threading
import time

import xml.etree.cElementTree as eT
from symbol import except_clause

ADDRESS = ('localhost', 12300)
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

        self.auth = eT.fromstring('''<?xml version="1.0" encoding="UTF-8" standalone="no"?>
            <message type="auth-request"><authentication password="test" username="test"/></message>''')
        self.message_id = -1

        self._pub_request_action = rospy.Publisher('~request_action', RequestAction, queue_size = 10)

        self._agent_topic_prefix = "agent_node_" + self.agent_name + "/"

        rospy.Subscriber(self._agent_topic_prefix + "generic_action", GenericAction, self.callback_generic_action)

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
            timestamp = message.get('timestamp')
            perception = message.find('perception')
            self.id = perception.get('id')
            print "request-action: perception id = ", self.id
            self.message_id = perception.get('id')
            # TODO why are self.id and self.message_id the same? is this correct?
            
            # TODO add the other publishers
            self._publish_request_action(timestamp=timestamp, perception=perception)
            #self.send(action_type="skip")

            
        elif typ == 'sim-start':
            print "sim-start: steps = ", message.find('simulation').get('steps')
        elif typ == 'auth-response':
            print "auth-response", message.find('authentication').get('result')
        elif typ == 'sim-end':
            print "ranking=", message.find('sim-result').get('ranking')
            print "score=", message.find('sim-result').get('score')
        elif typ == 'bye':
            print "Do somethingmeaningful on system exit"
            
    def send(self, action_type):
            response = eT.Element('message', type='action')
            action = eT.SubElement(response, 'action', id=self.message_id, type=action_type)
            self.s.send(eT.tostring(response) + SEPARATOR)

    def run(self):
        print "MacRosBridge::run"
        while not self.connect():
            time.sleep(RETRY_DELAY)
        self.authenticate()
        buffer = b''
        while (not rospy.is_shutdown()):
            try:
                data = self.s.recv(RECV_SIZE)
            except socket.timeout:
                print "socket timeout"
                self.reconnect()
                buffer = b''
                continue
            except socket.error as e:
                if e.errno == errno.ECONNRESET: #TODO check "errno" this is neither defined nor imported
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

    def callback_generic_action(self, msg):
        """
        :param msg: ros message
        :type msg: GenericAction
        """
        rospy.logdebug("MacRosBridge::callback_generic_action %s", msg)
        self.send(action_type=msg.action_type)

    def _publish_request_action(self, timestamp, perception):
        """
        :param timestamp: message timestamp
        :type timestamp: int
        :param perception: full perception object
        :type perception: eT  #TODO coorect?
        """
        msg = RequestAction()
        msg.timestamp = long(timestamp)
        msg.id = perception.get('id')
        msg.simulation_step = int(perception.find('simulation').get('step'))
        msg.deadline = long(perception.get('deadline'))
        rospy.loginfo("Request action %s", msg)
        self._pub_request_action.publish(msg)

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
