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

# Since we are at a different hierarchical level than ros1_pip_pytemplate,
# It is mandatory to install the module before testing ,
# which is good practice anyway, to make sure the install process also works.
#
# Works out of the box with catkin build,
# When running from pure python, use pip in a virtual environment !
# https://packaging.python.org/tutorials/installing-packages/#creating-virtual-environments


# Basic UnitTest TestCase
class TestHttpbin(unittest.TestCase):

    # fixture
    def setUp(self):
        self.httpbin = Httpbin('http://httpbin.org')

    def test_retrieve(self):
        resp = self.httpbin.get(params={"answer": "42"})
        status_code = resp.status_code
        json_data = resp.json()
        assert status_code == requests.status_codes.codes.OK
        assert json_data.get('origin') is not None
        assert json_data.get('url') is not None
        assert json_data.get('args') == {'answer': '42'}


# In case we run this by itself, outside of a testing framework like pytest
if __name__ == '__main__':
    pytest.main(['-s', '-x', __file__])
