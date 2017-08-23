from __future__ import absolute_import, division, print_function

"""
Test for lib_module
Reference : http://pythontesting.net/framework/nose/nose-introduction/
"""

import unittest
try:
    # Python >= 3.3
    import unittest.mock as mock
except ImportError:
    import mock as mock

import pytest

try:
    from io import StringIO
except ImportError:  # py27
    from StringIO import StringIO

import requests

# If we are calling pytest at a different hierarchical level than ros1_pip_pytemplate,
# we need first to install the module before testing,
# which is good practice anyway, to make sure the install process also works.
#
# Works out of the box with catkin build,
# When running from pure python, use pip in a virtual environment !
# https://packaging.python.org/tutorials/installing-packages/#creating-virtual-environments
# If access to ROS packages is needed (via pyros-setup) don't forget to enable site-packages.

# Here we are testing the core python code, importing only that module
from .. import Httpbin


# Basic UnitTest TestCase
class TestHttpbin(unittest.TestCase):

    # fixture
    def setUp(self):
        self.httpbin = Httpbin('http://httpbin.org')

    # The parent package has already been imported and loggers have been created.
    # Here we patch the existing logger to confirm message is being logged
    # Note there is also pytest-catchlog that can setup a pytest fixture for this...
    @mock.patch('ros1_pip_pytemplate.lib_module._logger')
    def test_retrieve(self, mock_logger):
        resp = self.httpbin.get(params={"answer": "42"})
        mock_logger.info.assert_called_once_with("Sending GET request to http://httpbin.org/get...")

        status_code = resp.status_code
        json_data = resp.json()
        assert status_code == requests.status_codes.codes.OK
        assert json_data.get('origin') is not None
        assert json_data.get('url') is not None
        assert json_data.get('args') == {'answer': '42'}


# In case we run this by itself, outside of a testing framework like pytest
if __name__ == '__main__':
    pytest.main(['-s', '-x', __file__])
