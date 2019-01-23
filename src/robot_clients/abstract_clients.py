from abc import ABCMeta, abstractmethod


class AbstractROSClient():
    __metaclass__ = ABCMeta

    @abstractmethod
    def close():
        pass

    @abstractmethod
    def call():
        pass