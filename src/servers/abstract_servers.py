from abc import ABCMeta, abstractmethod


class AbstractROSServer():
    __metaclass__ = ABCMeta

    @abstractmethod
    def close():
        pass

    @abstractmethod
    def call():
        pass