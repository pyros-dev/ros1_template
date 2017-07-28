from __future__ import absolute_import, division, print_function

"""
Module holding testable units of computation
"""


def answer():
    """Unit of testable functionality as a function"""
    return 42


class Answer(object):
    """Unit of testable functionality as a class"""
    def __init__(self):
        self.part = 6

    def retrieve(self):
        return 7*self.part


class Fibonacci(object):
    """Unit of testable functionality as an iterator"""
    def __init__(self, answer):
        self.n = 0
        self.m = 1

    def __iter__(self):
        return self

    def next(self):
        l = self.n + self.m
        self.n = self.m
        self.m = l
        return l
