from mapc_rhbp_ettlinger.msg import TaskCoordinationMessage

from rospy import Publisher, Subscriber


class MyPublisher(Publisher):
    """
    Publisher of rospy uses one thread for each Subscriber. In the case of this project, this results in many thousands
    of threads. Therefore this Publisher combines multiple topic, who then all share the same thread.
    """

    def __init__(self, name, message_type, task_type, subscriber_listener=None, tcp_nodelay=False, latch=False,
                 headers=None,
                 queue_size=None):
        super(MyPublisher, self).__init__(name, data_class=TaskCoordinationMessage,
                                          subscriber_listener=subscriber_listener, tcp_nodelay=tcp_nodelay,
                                          latch=latch, headers=headers, queue_size=queue_size)
        self._my_message_type = message_type
        self._my_task_type = task_type

    def publish(self, field):
        msg = TaskCoordinationMessage(message_type=self._my_message_type, task_type=self._my_task_type)
        setattr(msg, self._my_message_type, field)

        super(MyPublisher, self).publish(msg)


class MySubscriber(Subscriber):
    """
    Subscriber of rospy uses one thread for each Publisher. In the case of this project, this results in many thousands
    of threads. Therefore this Subscriber combines multiple topic, who then all share the same thread.
    """

    def __init__(self, name, callback, message_type, task_type, **kwargs):
        self.my_message_type = message_type
        self.my_task_type = task_type
        self.client_callback = callback
        super(MySubscriber, self).__init__(name, data_class=TaskCoordinationMessage, callback=self.my_callback,
                                           **kwargs)

    def my_callback(self, msg):
        """

        :param msg:
        :type msg: TaskCoordinationMessage
        :return:
        """

        if msg.task_type == self.my_task_type and msg.message_type == self.my_message_type:
            client_msg = getattr(msg, self.my_message_type, None)
            self.client_callback(client_msg)
        else:
            return None
