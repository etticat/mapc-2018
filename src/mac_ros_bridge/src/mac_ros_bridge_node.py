#!/usr/bin/env python2

# Thanks to Python-DTU for inspiration!
# Parts of code taken from Communicator class

import rospy

import traceback

from mac_ros_bridge.msg import RequestAction, GenericAction, Agent, AuctionJob, AuctionJobMsg, \
    ChargingStation, ChargingStationMsg, Dump, DumpMsg, Item, Job, JobMsg, Shop, ShopMsg, Storage, StorageMsg, Team, \
    Workshop, WorkshopMsg, Position, Entity, EntityMsg, Role, Resource, ResourceMsg, Bye, SimStart, SimEnd, \
    Product, Well, WellMsg, Upgrade

import socket
import threading
import time
import errno

import xml.etree.cElementTree as eT


class MacRosBridge(threading.Thread):
    """
    Proxy server that converts the communication with the Multi-Agent Programming Contest Server 2017
    to ROS topics
    """

    SOCKET_TIMEOUT = 2  # general socket timeout
    RETRY_DELAY = 1.0
    RECV_SIZE = 8192
    SEPARATOR = b'\0'

    def __init__(self, name=None):
        """
        :param name: agent name, leave empty to get name property from roslaunch
        """
        super(MacRosBridge, self).__init__()
        rospy.logdebug("MacRosBridge::init")

        node_name = 'bridge_node_'

        if name:
            node_name += name
        rospy.init_node(node_name, anonymous=False, log_level=rospy.INFO)

        if not name:
            self._agent_name = rospy.get_param('~agent_name', 'agentA1')
        else:
            self._agent_name = name

        server_ip = rospy.get_param('~server_ip', 'localhost')
        server_port = rospy.get_param('~server_port', 12300)
        self._server_address = (server_ip, server_port)

        self._agent_pw = rospy.get_param('~password', '1')

        # configure if also general information available for all agents should be handled
        self._only_agent_specific = rospy.get_param('~only_agent_specific', True)

        rospy.logdebug("Server: %s Port: %d Agent: %s Password: %s", server_ip, server_port, self._agent_name,
                       self._agent_pw)

        self._auth = eT.fromstring('''<?xml version="1.0" encoding="UTF-8" standalone="no"?>
            <message type="auth-request"><auth-request username="PLACEHOLDER" password="PLACEHOLDER"/></message>''')
        self.message_id = -1

        # this is all perception that the agent is going to publish

        self._pub_request_action = rospy.Publisher('~request_action', RequestAction, queue_size=1)
        self._pub_agent = rospy.Publisher('~agent', Agent, queue_size=1, latch=True)
        self._pub_sim_start = rospy.Publisher('~start', SimStart, queue_size=1, latch=True)
        self._pub_sim_end = rospy.Publisher('~end', SimEnd, queue_size=1, latch=False)
        self._pub_bye = rospy.Publisher('~bye', Bye, queue_size=1, latch=True)
        self._pub_local_wells = rospy.Publisher('~well', WellMsg, queue_size=1, latch=True)
        self._pub_local_entity = rospy.Publisher('~entity', EntityMsg, queue_size=1, latch=True)
        self._pub_global_wells = rospy.Publisher('/well', WellMsg, queue_size=1, latch=True)
        # self._pub_global_entity = rospy.Publisher('/entity', EntityMsg, queue_size=1, latch=True)
        self._pub_global_resource = rospy.Publisher('/resource', ResourceMsg, queue_size=1, latch=True)
        self._pub_local_resource = rospy.Publisher('~resource', ResourceMsg, queue_size=1, latch=True)

        if not self._only_agent_specific:
            self._pub_team = rospy.Publisher('/team', Team, queue_size=1, latch=True)
            self._pub_shop = rospy.Publisher('/shop', ShopMsg, queue_size=1, latch=True)
            self._pub_charging_station = rospy.Publisher('/charging_station', ChargingStationMsg, queue_size=1,
                                                         latch=True)
            self._pub_dump = rospy.Publisher('/dump', DumpMsg, queue_size=1, latch=True)
            self._pub_storage = rospy.Publisher('/storage', StorageMsg, queue_size=1, latch=True)
            self._pub_workshop = rospy.Publisher('/workshop', WorkshopMsg, queue_size=1, latch=True)
            self._pub_priced_job = rospy.Publisher('/priced_job', JobMsg, queue_size=1, latch=True)
            self._pub_auction_job = rospy.Publisher('/auction_job', AuctionJobMsg, queue_size=1, latch=True)
            self._pub_mission = rospy.Publisher('/mission_job', JobMsg, queue_size=1, latch=True)

        # this is the expected action reply
        rospy.Subscriber("~generic_action", GenericAction, self.callback_generic_action)

    def connect(self):
        """
        connect to contest server
        :return: bool True for success
        """
        try:
            rospy.logdebug("Connecting...%s", self._agent_name)
            self._socket = socket.create_connection(self._server_address, MacRosBridge.RETRY_DELAY)
            self._socket.settimeout(None) # enable blocking mode until simulation starts
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
            self._socket.shutdown(socket.SHUT_RDWR)
            self._socket.close()
        except OSError:
            pass
        time.sleep(MacRosBridge.RETRY_DELAY)
        rospy.loginfo('Reconnecting...%s', self._agent_name)
        self._socket = None
        while not self.connect():
            time.sleep(MacRosBridge.RETRY_DELAY)
        self.authenticate()

    def authenticate(self):
        """
        Authenticate on the contest server
        """
        rospy.logdebug("Authenticate...%s", self._agent_name)
        auth_element = self._auth.find('auth-request')
        auth_element.attrib['username'] = self._agent_name
        auth_element.attrib['password'] = self._agent_pw
        self._socket.send(eT.tostring(self._auth) + MacRosBridge.SEPARATOR)

    def handle_message(self, xml):
        """
        Handle server messages
        :param xml: xml message
        :type xml: eT ElementTree
        """
        try:
            message = eT.fromstring(xml)
            typ = message.get('type')
            if typ == 'request-action':
                self._request_action(message=message)
            elif typ == 'sim-start':
                self._sim_start(message=message)
            elif typ == 'auth-response':
                rospy.loginfo("%s: Authentication: %s", self._agent_name, message.find('auth-response').get('result'))
            elif typ == 'sim-end':
                self._sim_end(message=message)
            elif typ == 'bye':
                self._bye(message=message)
        except Exception as e:
            rospy.logerr(traceback.format_exc())

    def _request_action(self, message):
        """
        Handle request action message
        :param message: xml message
        """

        timestamp = long(message.get('timestamp'))
        perception = message.find('percept')

        self.message_id = perception.get('id')
        rospy.logdebug("request-action: perception id = %s", self.message_id)

        self._publish_agent(timestamp=timestamp, perception=perception)
        self._publish_entity(timestamp=timestamp, perception=perception)
        self._publish_well(timestamp=timestamp, perception=perception)
        self._publish_resources(timestamp=timestamp, perception=perception)

        if not self._only_agent_specific:
            self._publish_team(timestamp=timestamp, perception=perception)
            self._publish_facilities(timestamp=timestamp, perception=perception)
            self._publish_jobs(timestamp=timestamp, perception=perception)
        # self.send(action_type="recharge") # can be enabled for dummy answers

        # first send all updates before requesting the action
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
        msg.proximity = 1.0 / (10.0 ** int(simulation.get('proximity')))  # given proximity is converted to a more useful scale
        msg.cell_size = int(simulation.get('cellSize')) / 1000.0  # convert from meter to kilometer

        msg.min_lat = float(simulation.get('minLat'))
        msg.max_lat = float(simulation.get('maxLat'))
        msg.min_lon = float(simulation.get('minLon'))
        msg.max_lon = float(simulation.get('maxLon'))
        msg.center_lat = float(simulation.get('centerLat'))
        msg.center_lon = float(simulation.get('centerLon'))

        msg.products = self._parse_products(simulation)

        msg.upgrades = self._parse_upgrades(simulation)

        msg.wells = self._parse_wells(simulation)

        msg.role = self._parse_role(simulation)

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
        # add parameters, keys are not used in the moment but might be useful for debugging,type checks or future extensions
        for key, value in params.iteritems():
            p = eT.SubElement(action, 'p')
            p.text = value

        rospy.logdebug("Action: " + eT.tostring(response))
        self._socket.send(eT.tostring(response) + MacRosBridge.SEPARATOR)

    def run(self):
        """
        Agent main thread
        """
        rospy.logdebug("MacRosBridge::run")
        while not self.connect():
            time.sleep(MacRosBridge.RETRY_DELAY)
        #        self._socket.settimeout(MacRosBridge.SOCKET_TIMEOUT)
        self.authenticate()
        buffer = b''
        while (not rospy.is_shutdown()):
            try:
                data = self._socket.recv(MacRosBridge.RECV_SIZE)
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

    def _parse_products(self, simulation):
        products = []
        items = simulation.findall('item')
        for item in items:
            product = Product()
            product.name = item.get('name')
            product.volume = int(item.get('volume'))
            items = []
            roles = []
            for role_item_elm in item:
                if role_item_elm.tag == "item":
                    item = Item()
                    item.name = role_item_elm.get('name')
                    amount = role_item_elm.get('amount')
                    if amount:
                        item.amount = int(amount)
                    else:
                        item.amount = 1
                    items.append(item)
                elif role_item_elm.tag == "role":
                    roles.append(role_item_elm.get('name'))
                else:
                    rospy.logerr("Unknown item product '%s' ! %s", role_item_elm.tag, traceback.format_exc())
            product.consumed_items = items
            product.required_roles = roles
            products.append(product)
        return products

    def _parse_upgrades(self, simulation):
        upgrades = []
        upgrades_xml = simulation.findall('upgrade')
        for elem in upgrades_xml:
            upgrade = Upgrade()
            upgrade.cost = int(elem.get('cost'))
            upgrade.name = elem.get('name')
            upgrade.step = int(elem.get('step'))
            upgrades.append(upgrade)
        return upgrades

    def _parse_role(self, simulation):

        xml_role = simulation.find('role')

        # eT.dump(xml_role)

        role = Role()
        role.name = xml_role.get('name')
        role.base_speed = int(xml_role.get('baseSpeed'))
        role.max_speed = int(xml_role.get('maxSpeed'))
        role.base_battery = int(xml_role.get('baseBattery'))
        role.max_battery = int(xml_role.get('maxBattery'))
        role.base_load = int(xml_role.get('baseLoad'))
        role.max_load = int(xml_role.get('maxLoad'))
        role.base_vision = int(xml_role.get('baseVision'))
        role.max_vision = int(xml_role.get('maxVision'))
        role.base_skill = int(xml_role.get('baseSkill'))
        role.max_skill = int(xml_role.get('maxSkill'))
        # role.roads = str(xml_role.get('roads')) # TODO not yet sent by the server

        return role

    def _parse_entities(self, perception):
        entities = []
        for xml_item in perception.findall('entity'):
            entity = Entity()
            entity.name = xml_item.get('name')
            entity.team = xml_item.get('team')
            entity.role = xml_item.get('role')
            entity.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))
            entities.append(entity)
        return entities

    def _parse_resources(self, perception):
        resources = []
        for xml_item in perception.findall('resourceNode'):
            rospy.logdebug("Resource %s", eT.tostring(xml_item))
            resource = Resource()
            resource.name = xml_item.get('name')
            resource.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))

            item = Item()
            item.name = xml_item.get('resource')
            item.amount = 1  # We can always only get one item per time
            resource.item = item
            resources.append(resource)
        return resources

    def _parse_storages(self, perception):
        storages = []
        for xml_item in perception.findall('storage'):
            storage = Storage()
            storage.name = xml_item.get('name')
            storage.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))
            storage.total_capacity = int(xml_item.get('totalCapacity'))
            storage.used_capacity = int(xml_item.get('usedCapacity'))
            storage.items = self._get_items(elem=xml_item)
            storages.append(storage)
        return storages

    def _parse_workshops(self, perception):
        workshops = []
        for xml_item in perception.findall('workshop'):
            workshop = Workshop()
            workshop.name = xml_item.get('name')
            workshop.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))
            workshops.append(workshop)
        return workshops

    def _parse_wells(self, perception):
        wells = []
        for xml_item in perception.findall('well'):
            well = Well()
            well.name = xml_item.get('name')
            lat = xml_item.get('lat')
            lon = xml_item.get('lon')
            if lat and lon:
                well.pos = Position(float(lat), float(lon))
            cost = xml_item.get('cost')
            if cost:
                well.cost = int(cost)
            initial_integrity = xml_item.get('initialIntegrity')
            if initial_integrity:
                well.initial_integrity = int(initial_integrity)
            efficiency = xml_item.get('efficiency')
            if efficiency:
                well.efficiency = int(efficiency)
            well.integrity = int(xml_item.get('integrity'))
            wells.append(well)
        return wells

    def _parse_shops(self, perception):
        shops = []
        for xml_item in perception.findall('shop'):
            shop = Shop()
            shop.name = xml_item.get('name')
            shop.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))
            restock = xml_item.get('restock')
            if restock:  # TODO not always included in server message, potential bug
                shop.restock = int(restock)
            shop.items = self._get_items(elem=xml_item)
            shops.append(shop)
        return shops

    def _parse_team(self, perception, timestamp):
        team = perception.find('team')
        msg_team = Team()
        msg_team.massium = long(team.get('massium'))
        msg_team.score = long(team.get('score'))
        msg_team.timestamp = timestamp
        return msg_team

    def _parse_jobs(self, perception, identifier):
        jobs = []
        for xml_item in perception.findall(identifier):
            job = self._get_common_job(elem=xml_item)
            job.type = identifier
            jobs.append(job)
        return jobs

    def _parse_agent(self, perception, timestamp):
        agent_self = perception.find('self')
        msg = Agent()
        msg.timestamp = timestamp
        msg.charge = int(agent_self.get('charge'))
        msg.charge_max = int(agent_self.get('chargeMax'))
        msg.load = int(agent_self.get('load'))
        msg.load_max = int(agent_self.get('loadMax'))
        msg.skill = int(agent_self.get('skill'))
        msg.speed = int(agent_self.get('speed'))
        msg.vision = int(agent_self.get('vision'))
        msg.pos = Position(float(agent_self.get('lat')), float(agent_self.get('lon')))
        for agent_self_action in agent_self.iter('action'):
            msg.last_action = agent_self_action.get('type')
            msg.last_action_result = agent_self_action.get('result')
        msg.items = self._get_items_of_agent(elem=agent_self)

        facility = agent_self.get('facility')
        if facility:
            msg.facility = facility
            msg.in_facility = True
        else:
            msg.in_facility = False

        return msg

    def _get_items_of_agent(self, elem):

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
            item = self._parse_item(xml_item)
            items.append(item)
        return items

    def _parse_item(self, xml_item):
        """
        parsing a single xml item element
        :param xml_item:
        :return: Item()
        """
        item = Item()
        item.name = xml_item.get('name')
        amount = xml_item.get('amount')
        if amount:
            item.amount = int(amount)
        price = xml_item.get('price')
        if price:  # this is not always available
            item.price = int(price)
        delivered = xml_item.get('delivered')
        if delivered:  # this is not always available
            item.delivered = int(delivered)
        stored = xml_item.get('stored')
        if stored:  # this is not always available
            item.stored = int(stored)
        return item

    def _parse_auctions(self, perception):
        jobs = []

        for xml_item in perception.findall('auction'):

            job = AuctionJob()
            job.job = self._get_common_job(elem=xml_item)

            lowest_bid = xml_item.get('lowestBid')
            if lowest_bid:
                job.lowest_bid = int(lowest_bid)

            # TODO not sure if this is used/available at all
            max_bid = xml_item.get('maxBid')
            if max_bid:
                job.max_bid = max_bid

            auction_time = xml_item.get('auctionTime')
            if auction_time:
                job.auction_time = int(auction_time)
            jobs.append(job)

        return jobs

    def _parse_dumps(self, perception):
        dumps = []
        for xml_item in perception.findall('dump'):
            dump = Dump()
            dump.name = xml_item.get('name')
            dump.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))
            dumps.append(dump)
        return dumps

    def _parse_charging_stations(self, perception):
        charging_stations = []
        for xml_item in perception.findall('chargingStation'):
            cs = ChargingStation()
            cs.name = xml_item.get('name')
            cs.pos = Position(float(xml_item.get('lat')), float(xml_item.get('lon')))
            cs.rate = int(xml_item.get('rate'))
            charging_stations.append(cs)
        return charging_stations

    def _get_job_items(self, elem):
        """
        Extract required job items from an xml element
        :param elem: xml Element
        :return: list of Item
        """
        items = []

        for xml_item in elem.findall('required'):
            item = self._parse_item(xml_item)
            items.append(item)
        return items

    def _get_common_job(self, elem):
        """
        Extract common job from an xml element
        :param elem: xml Element
        :return: Job
        """
        job = Job()
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

        msg.priced_jobs = self._parse_jobs(perception, identifier='job')

        msg.mission_jobs = self._parse_jobs(perception, identifier='mission')

        msg.auction_jobs = self._parse_auctions(perception)

        msg.agent = self._parse_agent(perception, timestamp)

        msg.entities = self._parse_entities(perception)

        msg.team = self._parse_team(perception, timestamp)

        msg.shops = self._parse_shops(perception)

        msg.workshops = self._parse_workshops(perception)

        msg.storages = self._parse_storages(perception)

        msg.resources = self._parse_resources(perception)

        msg.dumps = self._parse_dumps(perception)

        #rospy.logdebug("Request action %s", msg)
        self._pub_request_action.publish(msg)

    def _publish_agent(self, timestamp, perception):
        """
        :param timestamp: message timestamp
        :type timestamp: int
        :param perception: full perception object
        :type perception: eT  ElementTree
        """
        if self._pub_agent.get_num_connections() > 0:
            msg = self._parse_agent(perception, timestamp)
            self._pub_agent.publish(msg)

    def _publish_team(self, timestamp, perception):
        """
        :param timestamp: message timestamp
        :type timestamp: int
        :param perception: full perception object
        :type perception: eT  ElementTree
        """
        if self._pub_team.get_num_connections() > 0:
            msg = self._parse_team(perception, timestamp)
            self._pub_team.publish(msg)

    def _publish_well(self, timestamp, perception):
        """
        :param timestamp: message timestamp
        :type timestamp: int
        :param perception: full perception object
        :type perception: eT  ElementTree
        """
        has_local_subscribers = self._pub_local_wells.get_num_connections() > 0
        has_global_subscribers = self._pub_global_wells.get_num_connections() > 0

        if has_local_subscribers or has_global_subscribers:
            well_msg = WellMsg()
            well_msg.facilities = self._parse_wells(perception)
            well_msg.timestamp = timestamp

            if has_local_subscribers:
                self._pub_local_wells.publish(well_msg)

            # publish only on the global topic if we have information
            if has_global_subscribers and len(well_msg.facilities) > 0:
                self._pub_global_wells.publish(well_msg)

    def _publish_entity(self, timestamp, perception):
        """
        :param timestamp: message timestamp
        :type timestamp: int
        :param perception: full perception object
        :type perception: eT  ElementTree
        """
        has_local_subscribers = self._pub_local_entity.get_num_connections() > 0
        # has_global_subscribers = self._pub_global_entity.get_num_connections() > 0

        if has_local_subscribers:
            entity_msg = EntityMsg()
            entity_msg.entities = self._parse_entities(perception)
            entity_msg.timestamp = timestamp

        if has_local_subscribers:
            self._pub_local_entity.publish(entity_msg)

        # publish only on the global topic if we have information
        # if has_global_subscribers and len(entity_msg.entities) > 0:
        #     self._pub_global_entity.publish(entity_msg)

    def _publish_resources(self, timestamp, perception):
        """
        :param timestamp: message timestamp
        :type timestamp: int
        :param perception: full perception object
        :type perception: eT  ElementTree
        """

        has_local_subscribers = self._pub_local_resource.get_num_connections() > 0
        has_global_subscribers = self._pub_global_resource.get_num_connections() > 0

        if has_local_subscribers or has_global_subscribers:
            msg = ResourceMsg()
            msg.timestamp = timestamp
            msg.facilities = self._parse_resources(perception)

        if has_local_subscribers:
            self._pub_local_resource.publish(msg)

        # publish only on the global topic if we have information
        if has_global_subscribers and len(msg.facilities) > 0:
            self._pub_global_resource.publish(msg)

    def _publish_facilities(self, timestamp, perception):
        """
        :param timestamp: message timestamp
        :type timestamp: int
        :param perception: full perception object
        :type perception: eT  ElementTree
        """
        if self._pub_shop.get_num_connections() > 0:
            msg = ShopMsg()
            msg.timestamp = timestamp
            msg.facilities = self._parse_shops(perception)
            self._pub_shop.publish(msg)

        if self._pub_workshop.get_num_connections() > 0:
            msg = WorkshopMsg()
            msg.timestamp = timestamp
            msg.facilities = self._parse_workshops(perception)
            self._pub_workshop.publish(msg)

        if self._pub_charging_station.get_num_connections() > 0:
            cs_msg = ChargingStationMsg()
            cs_msg.timestamp = timestamp
            cs_msg.facilities = self._parse_charging_stations(perception)
            self._pub_charging_station.publish(cs_msg)

        if self._pub_dump.get_num_connections() > 0:
            dump_msg = DumpMsg()
            dump_msg.facilities = self._parse_dumps(perception)
            dump_msg.timestamp = timestamp
            self._pub_dump.publish(dump_msg)

        if self._pub_storage.get_num_connections() > 0:
            msg = StorageMsg()
            msg.timestamp = timestamp
            msg.facilities = self._parse_storages(perception)
            self._pub_storage.publish(msg)

    def _publish_jobs(self, timestamp, perception):
        """
        :param timestamp: message timestamp
        :type timestamp: int
        :param perception: full perception object
        :type perception: eT  ElementTree
        """

        if self._pub_priced_job.get_num_connections() > 0:
            msg = JobMsg()
            msg.timestamp = timestamp
            msg.jobs = self._parse_jobs(perception, identifier='job')
            self._pub_priced_job.publish(msg)

        if self._pub_mission.get_num_connections() > 0:
            msg = JobMsg()
            msg.timestamp = timestamp
            msg.jobs = self._parse_jobs(perception, identifier='mission')
            self._pub_mission.publish(msg)

        if self._pub_auction_job.get_num_connections() > 0:
            msg = AuctionJobMsg()
            msg.timestamp = timestamp
            msg.jobs = self._parse_auctions(perception)
            self._pub_auction_job.publish(msg)


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
