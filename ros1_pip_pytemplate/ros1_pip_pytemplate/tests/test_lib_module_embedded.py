from __future__ import absolute_import, division, print_function

"""
Test for lib_module
Reference : http://pythontesting.net/framework/nose/nose-introduction/
"""
import unittest
import pytest

import requests

# Here we are testing the core python code, importing only that module
from .. import Httpbin

# Since we are at a different hierarchical level than ros1_template,
# It is mandatory to install the module first (which is good practice to make sure the install process also works)
# Works out of the box with catkin build,
# When running from pure python, use a virtual environment !


# Basic UnitTest TestCase
class TestHttpbin(unittest.TestCase):

    # fixture
    def setUp(self):
        self.httpbin = Httpbin('http://httpbin.org')

    def test_retrieve(self):
        status_code, json_data = self.httpbin.get(params={"answer": "42"})
        assert status_code == requests.status_codes.codes.OK
        assert json_data.get('origin') is not None
        assert json_data.get('url') is not None
        assert json_data.get('args') == {'answer': '42'}


# In case we run this by itself, outside of a testing framework like pytest
if __name__ == '__main__':
    pytest.main(['-s', '-x', __file__])
