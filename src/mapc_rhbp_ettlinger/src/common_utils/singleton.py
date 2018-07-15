class Singleton(type):
    """
    Singleton base class. Each class that uses this class as meta_class becomes a singleton automatically.
    """
    _instances = {}

    def __call__(cls, *args, **kwargs):
        """

        :param args:
        :param kwargs:
        :return:
        """

        # if the class requires an agent_name as constructor argument, make one instance per process per agent,
        # otherwise just one instance per process
        identifier = (cls, kwargs["agent_name"] if "agent_name" in kwargs else "universal")

        if identifier not in cls._instances:
            cls._instances[identifier] = super(Singleton, cls).__call__(*args, **kwargs)

        return cls._instances[identifier]
