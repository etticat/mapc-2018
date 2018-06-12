



class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        identifier = (cls, kwargs["agent_name"])
        if identifier not in cls._instances:
            cls._instances[identifier] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[identifier]