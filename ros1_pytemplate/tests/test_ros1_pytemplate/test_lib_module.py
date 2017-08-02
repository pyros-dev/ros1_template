from __future__ import absolute_import, division, print_function

"""
Test for lib_module
Reference : http://pythontesting.net/framework/nose/nose-introduction/
"""

import unittest

# Here we are testing the core python code, importing only that module
from ros1_pytemplate import lib_module

# Since we are at a different hierarchical level than ros1_template,
# It is mandatory to install the module first (which is good practice to make sure the install process also works)
# Works out of the box with catkin build,
# When running from pure python, use a virtual environment !


# Basic UnitTest TestCase
class TestAnswer(unittest.TestCase):

    # fixture
    def setUp(self):
        self.answer = lib_module.Answer(6)

    def test_retrieve(self):
        assert self.answer.retrieve() == 42


# More complex Unittest TestCase
class TestFibonacci(unittest.TestCase):

    # fixture
    def setUp(self):
        self.fib = lib_module.Fibonacci(0, 1, 17)

    def test_next(self):
        assert self.fib.next() == 1
        assert self.fib.next() == 2
        assert self.fib.next() == 3
        assert self.fib.next() == 5
        assert self.fib.next() == 8
        assert self.fib.next() == 13
        # assert self.fib.next() == 21  # we max at 17

        assert self.fib.next() == 1
        assert self.fib.next() == 1
        assert self.fib.next() == 2
        assert self.fib.next() == 3
        assert self.fib.next() == 5

    def test_next_again(self):
        assert self.fib.next() == 1
        assert self.fib.next() == 2
        assert self.fib.next() == 3
        assert self.fib.next() == 5
        assert self.fib.next() == 8
        assert self.fib.next() == 13
        # assert self.fib.next() == 21  # we max at 17

        assert self.fib.next() == 1
        assert self.fib.next() == 1
        assert self.fib.next() == 2
        assert self.fib.next() == 3
        assert self.fib.next() == 5


# In case we run this by itself, outside of a testing framework like nose
if __name__ == '__main__':
    unittest.main()
