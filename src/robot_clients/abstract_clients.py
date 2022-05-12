from abc import ABCMeta, abstractmethod


class AbstractROSClient(metaclass=ABCMeta):
    """
    Abstract ROS client class to enforce programming close() and call() interface.
    """

    @abstractmethod
    def close():
        pass

    @abstractmethod
    def call():
        pass