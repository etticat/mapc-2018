#!/usr/bin/env python2

# Thanks to Python-DTU for inspiration!
# Parts of code taken from Communicator class

import rospy

from mac_ros_bridge.msg import RequestAction, GenericAction, Agent, AuctionJob, \
    ChargingStation, Dump, Item, Job, Shop, Storage,Team, \
    Workshop, Position, Entity, Role, Resource, Bye, SimStart, SimEnd, Tool, Product

import socket
import threading
import time
import errno

import xml.etree.cElementTree as eT


class MacRosBridge (threading.Thread):
    """
    Proxy server that converts the communication with the Multi-Agent Programming Contest Server 2017
    to ROS topics
    """

    l general socket timeout
    RETRY_DELAY = 1.0
    RECV_SIZE = 8192
    SEPARATOR = b'\0'

    def __init__(self, name=None):
        """
        :param name: agent name, leave empty to get name from roslaunch
        """
        threading.Thread.__init__(self)
        rospy.logdebug("MacRosBridge::init")

        node_name = 'bridge_node_'

        if name:
            node_name += name
        rospy.init_node(node_name, anonymous=False, log_level=rospy.INFO)

        if not name:
            self._agent_name = rospy.get_param('~agent_name', 'UNKNOWN')
        else:
            self._agent_name = name

        server_ip = rospy.get_param('~server_ip', 'localhost')
        server_port = rospy.get_param('~server_port', 12300)
        self._server_address = (server_ip, server_port)

        self._agent_pw = rospy.get_param('~password', '1')

        # configure if also general information available for all agents should be handled
        self._only_agent_specific = rospy.get_param('~only_agent_specific', False)

        rospy.logdebug("Server: %s Port: %d Agent: %s Password: %s", server_ip, server_port, self._agent_name, self._agent_pw)

        self.auth = eT.fromstring('''<?xml version="1.0" encoding="UTF-8" standalone="no"?>
            <message type="auth-request"><auth-request username="PLACEHOLDER" password="PLACEHOLDER"/></message>''')
        self.message_id = -1

        self._pub_request_action = rospy.Publisher('~request_action', RequestAction, queue_size = 10)
        self._pub_agent = rospy.Publisher('~agent', Agent, queue_size = 1, latch=True)
        self._pub_sim_start = rospy.Publisher('~start', SimStart, queue_size=1, latch=True)
        self._pub_sim_end = rospy.Publisher('~end', SimEnd, queue_size=1, latch=True)
        self._pub_bye = rospy.Publisher('~bye', Bye, queue_size=1, latch=True)

        if not self._only_agent_specific:
            self._pub_team = rospy.Publisher('/team', Team, queue_size=1, latch=True)
            self._pub_entity = rospy.Publisher('/entity', Entity, queue_size=10, latch=False)
            self._pub_shop = rospy.Publisher('/shop', Shop, queue_size=100, latch=False)
            self._pub_item = rospy.Publisher('/item', Item, queue_size=10, latch=False)
            self._pub_charging_station = rospy.Publisher('/charging_station', ChargingStation, queue_size=10, latch=False)
            self._pub_dump = rospy.Publisher('/dump', Dump, queue_size=10, latch=False)
            self._pub_storage = rospy.Publisher('/storage', Storage, queue_size=10, latch=False)
            self._pub_workshop = rospy.Publisher('/workshop', Workshop, queue_size=10, latch=False)
            self._pub_priced_job = rospy.Publisher('/priced_job', Job, queue_size=5, latch=True)
            self._pub_posted_job = rospy.Publisher('/posted_job', Job, queue_size=5, latch=True)
            self._pub_auction_job = rospy.Publisher('/auction_job', AuctionJob, queue_size=5, latch=True)
            self._pub_mission = rospy.Publisher('/mission_job', Job, queue_size=5, latch=True)
            self._pub_resource = rospy.Publisher('/resource', Resource, queue_size=10, latch=True)


        rospy.Subscriber("~generic_action", GenericAction, self.callback_generic_action)

    def connect(self):
        """
        connect to contest server
        :return: bool True for success
        """
        try:
            rospy.logdebug("Connecting...%s", self._agent_name)
            self.socket = socket.create_connection(self._server_address, MacRosBridge.RETRY_DELAY)
            self.socket.settimeout(None) # enable blocking mode until simulation starts
            return True
        except OSError as error:
            rospy.logerr('OSError connecting to {}: {}'.format(self._server_address, error))
            return False
        except socket.timeout:
            rospy.logerr('socket.timeout: Error connecting to {}: {}'.format(self._server_address, "timeout"))
            return False
        except socket.error as e:
            rospy.logerr('socket.error: Error connecting to {}: {}'.format(self._server_address, e))
#            time.sleep(MacRosBridge.RETRY_DELAY)
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
        time.sleep(MacRosBridge.RETRY_DELAY)
        rospy.loginfo('Reconnecting...%s', self._agent_name)
        self.socket = None
        while not self.connect():
            time.sleep(MacRosBridge.RETRY_DELAY)
        self.authenticate()

    def authenticate(self):
        """
        Authenticate on the contest server
        """
        rospy.loginfo("Authenticate...%s", self._agent_name)
        auth_element = self.auth.find('auth-request')
        auth_element.attrib['username'] = self._agent_name
        auth_element.attrib['password'] = self._agent_pw
        self.socket.send(eT.tostring(self.auth) + MacRosBridge.SEPARATOR)

    def handle_message(self, xml):
        """
        Handle server messages
        :param xml: xml message
        :type xml: eT ElementTree
        """
        message = eT.fromstring(xml)
        typ = message.get('type')
        #print(xml)
        if typ == 'request-action':
            self._request_action(message=message)
        elif typ == 'sim-start':
            self._sim_start(message=message)
        elif typ == 'auth-response':
            rospy.logdebug("auth-response: %s", message.find('auth-response').get('result'))
        elif typ == 'sim-end':
            self._sim_end(message=message)
        elif typ == 'bye':
            self._bye(message=message)

    def _request_action(self, message):
        """
        Handle request action message
        :param message: xml message
        """
        timestamp = long(message.get('timestamp'))
        perception = message.find('percept')

        #eT.dump(perception)  # TODO remove this

        self.message_id = perception.get('id')
        rospy.logdebug("request-action: perception id = %s", self.message_id)

        self._publish_agent(timestamp=timestamp, perception=perception)

        if not self._only_agent_specific:
            self._publish_team(timestamp=timestamp, perception=perception)
            self._publish_entity(timestamp=timestamp, perception=perception) #TODO check if this is agent specific or not
            self._publish_facilities(timestamp=timestamp, perception=perception)
            self._publish_jobs(timestamp=timestamp, perception=perception)
            self._publish_resources(timestamp=timestamp, perception=perception)
        # self.send(action_type="skip") # can be enabled for dummy answers

        #first send all updates before requesting the action
        self._publish_request_action(timestamp=timestamp, perception=perception)

    def _sim_end(self, message):
        """
        Handle sim end message
        :param message: xml message
        """
        timestamp = long(message.get('timestamp'))
        sim_result = message.find('sim-result')
        ranking = int(sim_result.get('ranking'))
        score = int(sim_result.get('score'))

        rospy.logdebug("ranking= %d", ranking)
        rospy.logdebug("score= %d", score)

        msg = SimEnd()
        msg.ranking = ranking
        msg.score = score
        msg.timestamp = timestamp
        self._pub_sim_end.publish(msg)

    def _sim_start(self, message):
        """
        Handle sim start message
        :param message: xml message
        """
        #eT.dump(message)

        timestamp = long(message.get('timestamp'))
        simulation = message.find('simulation')
        steps = int(simulation.get('steps'))
        rospy.logdebug("sim-start: steps = %s", steps)

        msg = SimStart()
        msg.timestamp = timestamp
        msg.simulation_id = simulation.get('id')
        msg.steps = steps
        msg.team = simulation.get('team')
        msg.map = simulation.get('map')
        #msg.tools = list(msg.tools)
        #TODO not yet available in the msg
        #msg.proximity = simulation.get('proximity')
        products = []
        items = simulation.findall('item')
        for item in items:
            product = Product()
            product.name = item.get('name')
            product.volume = int(item.get('volume'))
            items = []
            tools = []
            for tool_item in item:
                if tool_item.tag != "tool":
                    item = Item()
                    item.name = tool_item.get('name')
                    item.amount = int(tool_item.get('amount'))
                    items.append(item)
                else:
                    tool = Tool()
                    tool.name = tool_item.get('name')
                    tools.append(tool)
            product.consumed_items = items
            product.required_tools = tools
            products.append(product)
        msg.products = products

        xml_role = simulation.find('role')
        role = Role()
        role.speed = int(xml_role.get('speed'))
        role.name = xml_role.get('name')
        role.max_battery = int(xml_role.get('battery'))
        role.max_load = int(xml_role.get('load'))
        tools = []
        for role_tool in xml_role.findall('tool'):
            tool = Tool()
            tool.name = role_tool.get('name')
            tools.append(tool)
        role.tools = tools
        msg.role = role
        #TODO add more information here from message content

        self._pub_sim_start.publish(msg)

    def _bye(self, message):
        """
        Handle bye message
        :param message: xml message
        """
        msg = Bye()
        timestamp = long(message.get('timestamp'))
        msg.timestamp = timestamp
        self._pub_bye.publish(msg)
        rospy.signal_shutdown('Shutting down {}  - Simulation server closed'.format(self._agent_name))

    def _send_action(self, action_type, params={}):
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

        rospy.logdebug("Action: " + eT.tostring(response))
        self.socket.send(eT.tostring(response) + MacRosBridge.SEPARATOR)

    def run(self):
        """
        Agent main thread
        """
        rospy.logdebug("MacRosBridge::run")
        while not self.connect():
            time.sleep(MacRosBridge.RETRY_DELAY)
        #self.socket.settimeout(MacRosBridge.SOCKET_TIMEOUT)
        self.authenticate()
        buffer = b''
        while (not rospy.is_shutdown()):
            try:
                data = self.socket.recv(MacRosBridge.RECV_SIZE)
            except socket.timeout:
                rospy.logerr("socket timeout")
                self.reconnect()
                buffer = b''
                continue
            except socket.error as e:
                if e.errno == errno.ECONNRESET:
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
                index = buffer.find(MacRosBridge.SEPARATOR)
                while index != -1:
                    self.handle_message(buffer[0:index].decode())
                    buffer = buffer[index + 1:]
                    index = buffer.find(MacRosBridge.SEPARATOR)

    def callback_generic_action(self, msg):
        """
        ROS callback for generic actions
        :param msg: ros message
        :type msg: GenericAction
        """
        rospy.logdebug("MacRosBridge::callback_generic_action %s", msg)

        params = {}
        for key_value in msg.params:
            params[key_value.key] = key_value.value

        self._send_action(action_type=msg.action_type, params=params)

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
        rospy.logdebug("Request action %s", msg)
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
            for agent_self_action in agent_self.iter('action'):

                msg.last_action = agent_self_action.get('type')
                msg.last_action_result = agent_self_action.get('result')
            #msg.route_length = int(agent_self.get('routeLength')) TODO might not be available anymore

            msg.items = self._get_items_of_agent(elem=agent_self)

            self._pub_agent.publish(msg)

    def _get_items_of_agent(self,elem):

        """
        Extract Items from an agent xml element
        :param elem: agent xml Element
        :return: list of Item
        """
        items = []

        for xml_item in elem.findall('items'):
            item = Item()
            item.name = xml_item.get('name')
            item.amount = int(xml_item.get('amount'))

            items.append(item)
        return items

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
            amount = xml_item.get('amount')
            if amount:
                item.amount = int(amount)
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

    def _get_job_items(self, elem):
        """
        Extract required job items from an xml element
        :param elem: xml Element
        :return: list of Item
        """
        items = []

        for xml_item in elem.findall('required'):
            item = Item()
            item.name = xml_item.get('name')
            item.amount = int(xml_item.get('amount'))
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

            for xml_item in perception.findall('entity'):
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

        if self._pub_shop.get_num_connections() > 0:
            for xml_item in perception.findall('shop'):

                shop = Shop()
                shop.timestamp = timestamp
                shop.name = xml_item.get('name')
                shop.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))
                restock = xml_item.get('restock')
                if restock: #TODO not always included in server message, potential bug
                    shop.restock = int(restock)
                shop.items = self._get_items(elem=xml_item)
                self._pub_shop.publish(shop)

	#TODO could be combined with the former loop
        if self._pub_item.get_num_connections() > 0:
            amount_items = []
            for xml_item in perception.iter('shop'):
                for xml_item_child in xml_item.iter('item'):
                    item = Item()
                    amount = xml_item_child.get('amount')
                    amount_items.append(int(xml_item_child.get('amount')))
                    if amount:
                        item.amount = int(amount)
                    item.name = xml_item_child.get('name')
                    price = xml_item_child.get('price')
                    if price:
                        item.price = int(price)
                    #shop.items = self._get_items(elem=xml_item)
                sum(amount_items)
                self._pub_item.publish(item)


        if self._pub_workshop.get_num_connections() > 0:
            for xml_item in perception.findall('workshop'):
                workshop = Workshop()
                workshop.timestamp = timestamp
                workshop.name = xml_item.get('name')
                workshop.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))
                self._pub_workshop.publish(workshop)

        if self._pub_charging_station.get_num_connections() > 0:
            for xml_item in perception.findall('chargingStation'):
                cs = ChargingStation()
                cs.timestamp = timestamp
                cs.name = xml_item.get('name')
                cs.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))
                cs.rate = int(xml_item.get('rate'))
                self._pub_charging_station.publish(cs)

        if self._pub_dump.get_num_connections() > 0:
            for xml_item in perception.findall('dump'):
                dump = Dump()
                dump.timestamp = timestamp
                dump.name = xml_item.get('name')
                dump.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))
                self._pub_dump.publish(dump)

        if self._pub_storage.get_num_connections() > 0:
            for xml_item in perception.findall('storage'):
                storage = Storage()
                storage.timestamp = timestamp
                storage.name = xml_item.get('name')
                storage.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))
                storage.total_capacity = int(xml_item.get('totalCapacity'))
                storage.used_capacity = int(xml_item.get('usedCapacity'))
                storage.items = self._get_items(elem=xml_item)
                self._pub_storage.publish(storage)



    def _get_common_job(self, elem, timestamp):
        """
        Extract common job from an xml element
        :param elem: xml Element
        :return: Job
        """
        job = Job()
        job.timestamp = timestamp
        job.id = elem.get('id')
        job.storage_name = elem.get('storage')
        job.reward = int(elem.get('reward'))
        job.start = int(elem.get('start'))
        job.end = int(elem.get('end'))
        fine = elem.get('fine')
        if fine:
            job.fine = int(fine)
        job.items = self._get_job_items(elem=elem)

        return job

    def _publish_jobs(self, timestamp, perception):
        """
        :param timestamp: message timestamp
        :type timestamp: int
        :param perception: full perception object
        :type perception: eT  ElementTree
        """

        if self._pub_posted_job.get_num_connections() > 0:
            for xml_item in perception.findall('posted'):
                job = self._get_common_job(elem=xml_item, timestamp=timestamp)
                self._pub_posted_job.publish(job)

        if self._pub_priced_job.get_num_connections() > 0:
            for xml_item in perception.findall('job'):
                job = self._get_common_job(elem=xml_item, timestamp=timestamp)
                self._pub_priced_job.publish(job)

        if self._pub_mission.get_num_connections() > 0:
            for xml_item in perception.findall('mission'):
                job = self._get_common_job(elem=xml_item, timestamp=timestamp)
                self._pub_mission.publish(job)

        if self._pub_auction_job.get_num_connections() > 0:
            for xml_item in perception.findall('auction'):
                #rospy.loginfo("Auction")
                #eT.dump(xml_item)

                job = AuctionJob()
                job.job = self._get_common_job(elem=xml_item, timestamp=timestamp)

                lowest_bid = xml_item.get('lowestBid')
                if lowest_bid:
                    job.lowest_bid= int(lowest_bid)

                # TODO not sure if this is used/available at all
                max_bid = xml_item.get('maxBid')
                if max_bid:
                    job.max_bid = max_bid

                auction_time = xml_item.get('auctionTime')
                if auction_time:
                    job.auction_time = int(auction_time)

                    self._pub_auction_job.publish(job)
    
    def _publish_resources(self, timestamp, perception):
        """
        :param timestamp: message timestamp
        :type timestamp: int
        :param perception: full perception object
        :type perception: eT  ElementTree
        """
        if self._pub_resource.get_num_connections() > 0:
            for xml_item in perception.findall('resourceNode'):
                rospy.logdebug("Resource %s", eT.tostring(xml_item))
                resource = Resource()
                resource.timestamp = timestamp
                resource.name = xml_item.get('name')
                resource.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))

                item = Item()
                item.name = xml_item.get('resource')
                item.amount = 1 # TODO may use another number her
                resource.items.append(item)
                self._pub_resource.publish(resource)


if __name__ == '__main__':
    rospy.logdebug("mac_ros_bridge_node::main")
    try:
        bridge = MacRosBridge().start()
        # bridge = MacRosBridge("agentA1").start()
        # bridge = MacRosBridge("agentA2").start()
        # bridge = MacRosBridge("agentA3").start()
        # bridge = MacRosBridge("agentA4").start()
        # time.sleep(MacRosBridge.RETRY_DELAY)
        # bridge = MacRosBridge("agentA5").start()
        # bridge = MacRosBridge("agentA6").start()
        # bridge = MacRosBridge("agentA7").start()
        # bridge = MacRosBridge("agentA8").start()
        # time.sleep(MacRosBridge.RETRY_DELAY)
        # bridge = MacRosBridge("agentA9").start()
        # bridge = MacRosBridge("agentA10").start()
        # bridge = MacRosBridge("agentA11").start()
        # bridge = MacRosBridge("agentA12").start()
        # time.sleep(MacRosBridge.RETRY_DELAY)
        # bridge = MacRosBridge("agentA13").start()
        # bridge = MacRosBridge("agentA14").start()
        # bridge = MacRosBridge("agentA15").start()
        # bridge = MacRosBridge("agentA16").start()
        # time.sleep(MacRosBridge.RETRY_DELAY)
        #
        # bridge = MacRosBridge("agentB1").start()
        # bridge = MacRosBridge("agentB2").start()
        # bridge = MacRosBridge("agentB3").start()
        # bridge = MacRosBridge("agentB4").start()
        # time.sleep(MacRosBridge.RETRY_DELAY)
        # bridge = MacRosBridge("agentB5").start()
        # bridge = MacRosBridge("agentB6").start()
        # bridge = MacRosBridge("agentB7").start()
        # bridge = MacRosBridge("agentB8").start()
        # time.sleep(MacRosBridge.RETRY_DELAY)
        # bridge = MacRosBridge("agentB9").start()
        # bridge = MacRosBridge("agentB10").start()
        # bridge = MacRosBridge("agentB11").start()
        # bridge = MacRosBridge("agentB12").start()
        # time.sleep(MacRosBridge.RETRY_DELAY)
        # bridge = MacRosBridge("agentB13").start()
        # bridge = MacRosBridge("agentB14").start()
        # bridge = MacRosBridge("agentB15").start()
        # bridge = MacRosBridge("agentB16").start()
        # time.sleep(MacRosBridge.RETRY_DELAY)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
