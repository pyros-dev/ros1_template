#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# License: MIT
#

from __future__ import absolute_import, division, print_function


"""
Testing the cli
"""

##############################################################################
# Imports
##############################################################################

import contextlib
import sys  # late import to avoid breaking capsys fixture
import os
import runpy
import pytest


# a simple selftest for pytest capsys fixture
def test_capsys(capsys):
    print('smthg')
    sys.stderr.write('smthgelse')
    out, err = capsys.readouterr()
    assert 'smthg' in out
    assert 'smthgelse' in err

##############################################################################
# Test Class
##############################################################################


class TestCLI(object):  # not a unittest.TestCase, since we rely on pytest capsys fixture

    def setup(self):
        self.cli_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'scripts', 'cli.py')

    def test_help(self, capsys):
        import sys  # late import to avoid breaking capsys fixture
        # redirecting stdout and stderr since we are testing a script running on command line
        sys.argv = ['', '--help']
        with pytest.raises(SystemExit) as excinfo:
            runpy.run_path(self.cli_path, run_name='__main__')

        assert excinfo.value.code == 0  # success

        out, err = capsys.readouterr()

        # Note other output can get mixed here (internal loggers propagated upwards to the top)
        # We only want to assert a subset of the output
        rel_script_path = os.path.relpath(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'scripts', 'cli.py'))
        assert "usage: " + rel_script_path + " [-h|--help] [--version]" in out

    def test_version(self, capsys):
        # redirecting stdout and stderr since we are testing a script running on command line
        sys.argv = ['', '--version']
        with pytest.raises(SystemExit) as excinfo:
            runpy.run_path(self.cli_path, run_name='__main__')

        assert excinfo.value.code == 0  # success

        out, err = capsys.readouterr()

        # Note other output can get mixed here (internal loggers propagated upwards to the top)
        # We only want to assert a subset of the output
        assert "ROS1 pip pytemplate version 0.1.1" in out

    def test_noargs(self, capsys):
        # redirecting stdout and stderr since we are testing a script running on command line
        sys.argv = ['']
        runpy.run_path(self.cli_path, run_name='__main__')

        out, err = capsys.readouterr()

        # Note other output can get mixed here (internal loggers propagated upwards to the top)
        # We only want to assert a subset of the output
        assert "STATUS: 200" in out
        assert "args: {}" in out
        assert "origin: " in out  # origin will depend on machine
        assert "url: http://httpbin.org/get" in out

    def test_args(self, capsys):
        # redirecting stdout and stderr since we are testing a script running on command line
        sys.argv = ['', '--arg1', 'val1', '--arg2', 'val2']
        runpy.run_path(self.cli_path, run_name='__main__')

        out, err = capsys.readouterr()

        # Note other output can get mixed here (internal loggers propagated upwards to the top)
        # We only want to assert a subset of the output
        assert "STATUS: 200" in out
        assert "args: {arg1: val1, arg2: val2}" in out
        assert "origin: " in out  # origin will depend on machine
        assert "url: http://httpbin.org/get?arg1=val1&arg2=val2" or "url: http://httpbin.org/get?arg2=val2&arg1=val1" in out

    def test_bad_arg(self, capsys):
        # redirecting stdout and stderr since we are testing a script running on command line
        sys.argv = ['', '-badarg']
        with pytest.raises(SystemExit) as excinfo:
            runpy.run_path(self.cli_path, run_name='__main__')

        assert excinfo.value.code == 127  # error

        out, err = capsys.readouterr()

        # Note other output can get mixed here (internal loggers propagated upwards to the top)
        # We only want to assert a subset of the output
        rel_script_path = os.path.relpath(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'scripts', 'cli.py'))
        assert "usage: " + rel_script_path + " [-h|--help] [--version]" in out
        assert "Invalid Argument: -badarg" in err

# In case we run this directly, use pytest
if __name__ == '__main__':
    pytest.main(['-x', __file__])
