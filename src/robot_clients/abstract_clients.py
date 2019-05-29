from abc import ABCMeta, abstractmethod


class AbstractROSClient():
    """
    Abstract ROS client class to enforce programming close() and call() interface.
    """
    __metaclass__ = ABCMeta

    @abstractmethod
    def close():
        pass

    @abstractmethod
    def call():
        pass