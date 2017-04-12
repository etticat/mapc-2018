#!/usr/bin/env python2

# Thanks to Python-DTU for inspiration!
# Parts of code taken from Communicator class

import rospy

from mac_ros_bridge.msg import RequestAction, GenericAction, Agent, AuctionJob, \
    ChargingStation, Dump, Item, PricedJob, Product, Shop, Storage,Team, \
    Workshop, Position, Entity, Role

import socket
import threading
import time

import xml.etree.cElementTree as eT

ADDRESS = ('localhost', 12300)
RETRY_DELAY = 1.0
RECV_SIZE = 8192

PASSWORD = "1"

SEPARATOR = b'\0'

class MacRosBridge (threading.Thread):
    def __init__(self, name):
        """
        :param name: agent name
        """
        threading.Thread.__init__(self)
        rospy.loginfo("MacRosBridge::init")

        rospy.init_node('mac_ros_bridge_node', anonymous=True)

        self.agent_name = rospy.get_param('~agent_name', 'UNKNOWN')

        self.auth = eT.fromstring('''<?xml version="1.0" encoding="UTF-8" standalone="no"?>
            <message type="auth-request"><authentication password="test" username="test"/></message>''')
        self.message_id = -1

        self._pub_request_action = rospy.Publisher('~request_action', RequestAction, queue_size = 10)
        self._pub_agent = rospy.Publisher('~agent', Agent, queue_size = 10, latch=True)
        self._pub_team = rospy.Publisher('~team', Team, queue_size=10, latch=True)
        self._pub_entity = rospy.Publisher('/entity', Entity, queue_size=10, latch=True)
        self._pub_shop = rospy.Publisher('/shop', Shop, queue_size=10, latch=True)
        self._pub_charging_station = rospy.Publisher('/charging_station', ChargingStation, queue_size=10, latch=True)
        self._pub_dump = rospy.Publisher('/dump', Dump, queue_size=10, latch=True)
        self._pub_storage = rospy.Publisher('/storage', Storage, queue_size=10, latch=True)
        self._pub_workshop = rospy.Publisher('/workshop', Workshop, queue_size=10, latch=True)

        self._agent_topic_prefix = "agent_node_" + self.agent_name + "/"

        rospy.Subscriber(self._agent_topic_prefix + "generic_action", GenericAction, self.callback_generic_action)

    def connect(self):
        """
        connect to contest server
        :return: bool True for success
        """
        try:
            rospy.loginfo("Connecting...%s", self.agent_name)
            self.socket = socket.create_connection(ADDRESS, RETRY_DELAY)
            self.socket.settimeout(None) # enable blocking mode until simulation starts
            return True
        except OSError as error:
            rospy.logerr('Error connecting to {}: {}'.format(ADDRESS, error))
            return False
        except socket.timeout:
            rospy.logerr('Error connecting to {}: {}'.format(ADDRESS, "timeout"))
            return False
        except socket.error as e:
            rospy.logerr('Error connecting to {}: {}'.format(ADDRESS, e))
            return False
        
    def reconnect(self):
        """
        Reconnect to contest server
        """
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
            self.socket.close()
        except OSError:
            pass
        time.sleep(RETRY_DELAY)
        rospy.loginfo('Reconnecting...')
        self.socket = None
        while not self.connect():
            time.sleep(RETRY_DELAY)
        self.authenticate()
        
    def authenticate(self):
        """
        Autheticate on the contest server
        """
        auth_element = self.auth.find('authentication')
        auth_element.attrib['username'] = self.agent_name
        auth_element.attrib['password'] = PASSWORD
        self.socket.send(eT.tostring(self.auth) + SEPARATOR)
        
    def handle_message(self, xml):
        """
        Handle server messages
        :param xml: xml message
        :type xml: eT ElementTree
        """
        message = eT.fromstring(xml)
        typ = message.get('type')

        if typ == 'request-action':
            timestamp = long(message.get('timestamp'))
            perception = message.find('perception')

            self.message_id = perception.get('id')
            rospy.logdebug("request-action: perception id = ", self.message_id)

            # TODO add the other publishers
            self._publish_request_action(timestamp=timestamp, perception=perception)
            self._publish_agent(timestamp=timestamp, perception=perception)
            self._publish_team(timestamp=timestamp, perception=perception)
            self._publish_entity(timestamp=timestamp, perception=perception)
            self._publish_facilities(timestamp=timestamp, perception=perception)
            #self.send(action_type="skip")

            
        elif typ == 'sim-start':
            rospy.loginfo("sim-start: steps = %s", message.find('simulation').get('steps'))
        elif typ == 'auth-response':
            rospy.loginfo("auth-response: %s", message.find('authentication').get('result'))
        elif typ == 'sim-end':
            rospy.loginfo("ranking= %s", message.find('sim-result').get('ranking'))
            rospy.loginfo("score= %s", message.find('sim-result').get('score'))
        elif typ == 'bye':
            rospy.loginfo("Do somethingmeaningful on system exit")
            
    def send_action(self, action_type, params={}):
        """
        send action reply
        :param action_type: action type that should be executed
        :param params: dictionary with optional parameters, depending on the action type
        :return:
        """
        response = eT.Element('message', type='action')
        action = eT.SubElement(response, 'action', id=self.message_id, type=action_type)
        #add parameters, keys are not used in the moment but might be useful for debugging,type checks or future extensions
        for key, value in params.iteritems():
            p = eT.SubElement(action, 'p')
            p.text = value
        self.socket.send(eT.tostring(response) + SEPARATOR)

    def run(self):
        """
        Agent main thread
        """
        rospy.logdebug("MacRosBridge::run")
        while not self.connect():
            time.sleep(RETRY_DELAY)
        self.authenticate()
        buffer = b''
        while (not rospy.is_shutdown()):
            try:
                data = self.socket.recv(RECV_SIZE)
            except socket.timeout:
                rospy.logerr("socket timeout")
                self.reconnect()
                buffer = b''
                continue
            except socket.error as e:
                if e.errno == errno.ECONNRESET: #TODO check "errno" this is neither defined nor imported
                    # connection closed by server
                    data = b''
                else:
                    rospy.logerr(('Socket error: {}'.format(e)))
                    self.reconnect()
                    buffer = b''
                    continue
            if len(data) == 0:
                rospy.logerr(('Connection closed by server'))
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
        ROS callback for generic actions
        :param msg: ros message
        :type msg: GenericAction
        """
        rospy.logdebug("MacRosBridge::callback_generic_action %s", msg)

        params = {}
        for key, value in msg.params:
            params[key] = value

        self.send_action(action_type=msg.action_type, params=params)

    def _publish_request_action(self, timestamp, perception):
        """
        :param timestamp: message timestamp
        :type timestamp: long
        :param perception: full perception object
        :type perception: eT  ElementTree
        """
        msg = RequestAction()
        msg.timestamp = timestamp
        msg.id = perception.get('id')
        msg.simulation_step = int(perception.find('simulation').get('step'))
        msg.deadline = long(perception.get('deadline'))
        rospy.loginfo("Request action %s", msg)
        self._pub_request_action.publish(msg)

    def _publish_agent(self, timestamp, perception):
        """
        :param timestamp: message timestamp
        :type timestamp: int
        :param perception: full perception object
        :type perception: eT  ElementTree
        """
        if self._pub_agent.get_num_connections() > 0:
            agent_self = perception.find('self')

            msg = Agent()
            msg.timestamp = timestamp
            msg.charge = int(agent_self.get('charge'))
            msg.load = int(agent_self.get('load'))
            msg.pos = Position(float(agent_self.get('lat')), float(agent_self.get('lon')))
            msg.route_length = int(agent_self.get('routeLength'))

            msg.items = self._get_items(elem=agent_self)

            self._pub_agent.publish(msg)

    def _get_items(self, elem):
        """
        Extract Items from an xml element
        :param elem: xml Element
        :return: list of Item
        """
        items = []

        for xml_item in elem.findall('item'):
            item = Item()
            item.name = xml_item.get('name')
            item.amount = int(xml_item.get('amount'))
            price = xml_item.get('price')
            if price: #this is not always available
                item.price = int(price)
            delivered = xml_item.get('delivered')
            if delivered:  # this is not always available
                item.delivered = int(delivered)
            stored = xml_item.get('stored')
            if stored:  # this is not always available
                item.stored = int(stored)

            items.append(item)
        return items


    def _publish_team(self, timestamp, perception):
        """
        :param timestamp: message timestamp
        :type timestamp: int
        :param perception: full perception object
        :type perception: eT  ElementTree
        """
        if self._pub_team.get_num_connections() > 0:
            team = perception.find('team')

            msg = Team()
            msg.money = int(team.get('money'))
            msg.timestamp = timestamp
            self._pub_team.publish(msg)

    def _publish_entity(self, timestamp, perception):
        """
        :param timestamp: message timestamp
        :type timestamp: int
        :param perception: full perception object
        :type perception: eT  ElementTree
        """
        if self._pub_entity.get_num_connections() > 0:
            entities = perception.find('entities')

            for xml_item in entities.findall('entity'):
                entity = Entity()
                entity.name = xml_item.get('name')
                entity.team = xml_item.get('team')
                entity.role = Role(name=xml_item.get('role'))
                entity.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))
                entity.timestamp = timestamp

            self._pub_entity.publish(entity)


    def _publish_facilities(self, timestamp, perception):
        """
        :param timestamp: message timestamp
        :type timestamp: int
        :param perception: full perception object
        :type perception: eT  ElementTree
        """
        entities = perception.find('facilities')

        if self._pub_shop.get_num_connections() > 0:
            for xml_item in entities.findall('shop'):
                eT.dump(xml_item)
                shop = Shop()
                shop.timestamp = timestamp
                shop.name = xml_item.get('name')
                shop.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))
                restock = xml_item.get('restock')
                if restock: #TODO not always included in server message, potential bug
                    shop.restock = int(restock)
                shop.items = self._get_items(elem=xml_item)
                self._pub_shop.publish(shop)

        if self._pub_workshop.get_num_connections() > 0:
            for xml_item in entities.findall('workshop'):
                workshop = Workshop()
                workshop.timestamp = timestamp
                workshop.name = xml_item.get('name')
                workshop.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))
                self._pub_workshop.publish(workshop)

        if self._pub_charging_station.get_num_connections() > 0:
            for xml_item in entities.findall('chargingStation'):
                cs = ChargingStation()
                cs.timestamp = timestamp
                cs.name = xml_item.get('name')
                cs.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))
                cs.rate = int(xml_item.get('rate'))
                cs.slots = int(xml_item.get('slots')) #TODO available in server message but not clear if still used 2017
                #TODO charging stations also still has a price, contradicts the changelog
                self._pub_charging_station.publish(cs)

        if self._pub_dump.get_num_connections() > 0:
            for xml_item in entities.findall('dump'):
                dump = Dump()
                dump.timestamp = timestamp
                dump.name = xml_item.get('name')
                dump.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))
                self._pub_dump.publish(dump)

        if self._pub_storage.get_num_connections() > 0:
            for xml_item in entities.findall('storage'):
                storage = Storage()
                storage.timestamp = timestamp
                storage.name = xml_item.get('name')
                storage.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))
                storage.total_capacity = int(xml_item.get('totalCapacity'))
                storage.used_capacity = int(xml_item.get('usedCapacity'))
                storage.items = self._get_items(elem=xml_item)
                self._pub_storage.publish(storage)



if __name__ == '__main__':
    rospy.logdebug("mac_ros_bridge_node::main")
    try:
        bridge = MacRosBridge(name=rospy.get_param('~agent_name', 'UNKNOWN')).start()
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
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
