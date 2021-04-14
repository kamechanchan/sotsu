#!/usr/bin/env python

import abc


class Behavior:
    """define bahavior abstruct class"""

    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def execute_impl(self):
        raise NotImplementedError()
