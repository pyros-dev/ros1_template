from __future__ import absolute_import, division, print_function

import sys

# importing a pure python package
# To be able to get ros1_pip_pytemplate packaged on ROS, first one would need to release a ROS package for requests
# Using catkin_pip for this is probably the easiest solution.
import requests

"""
Module holding calls to a python package retrieved by pip
"""

import logging
_logger = logging.getLogger(__name__)
_logger.addHandler(logging.NullHandler())


class Httpbin(object):
    """Unit of testable functionality as a class, calling functionality from other libraries"""
    def __init__(self, base_url=None):
        self.basename = base_url or 'http://httpbin.org'
        _logger.debug("Httpbin proxy was initialized!")

    def get(self, headers=None, params=None):
        return requests.get('http://httpbin.org/get', headers=headers or {}, params=params or {})

    # Note a more complete version of this is available there :
    # https://github.com/asmodehn/pyros-schemas-examples/tree/master/pyros_httpbin
