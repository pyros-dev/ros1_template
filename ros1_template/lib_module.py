from __future__ import absolute_import, division, print_function

import sys

"""
Module holding testable units of computation
"""

import logging
_logger = logging.getLogger(__name__)
_logger.addHandler(logging.NullHandler())


class Answer(object):
    """Unit of testable functionality as a class"""
    def __init__(self, answer_portion):
        self.part = answer_portion
        _logger.debug("The Answer was initialized!")

    def retrieve(self):
        _logger.debug("The Answer is being computed...")
        return 7*self.part


class Fibonacci(object):
    """Unit of testable functionality as an iterator"""
    def __init__(self, init_first, init_second, max=256):
        self.n = init_first
        self.m = init_second
        self.max = max
        _logger.debug("The Fibonacci was initialized with {0} and {1}".format(self.n, self.m))

    def __iter__(self):
        return self

    def next(self):
        _logger.debug("The next Fibonacci number is being computed...")

        if self.n + self.m > self.max:
            # wrap around to avoid overflow
            l = 1
            self.n = 0
        else:
            l = self.n + self.m
            self.n = self.m
        self.m = l

        _logger.debug("The next Fibonacci number is {0}".format(l))

        return l
