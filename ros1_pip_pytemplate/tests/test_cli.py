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
        runpy.run_path(self.cli_path)

        #outlines = sys.stdout.readlines()

        # Note other output can get mixed here (internal loggers propagated upwards to the top)
        # We only want to assert a subset of the output
        # assert "" in outlines, outlines
        # assert "" in outlines, outlines
        # assert "" in outlines, outlines

    def test_version(self, capsys):
        # redirecting stdout and stderr since we are testing a script running on command line
        sys.argv = ['', '--version']
        with pytest.raises(SystemExit) as excinfo:
            runpy.run_path(self.cli_path, run_name='__main__')

        assert excinfo.value.code == 0  # success
        assert excinfo.value.message == excinfo.value.code  # success

        out, err = capsys.readouterr()

        # Note other output can get mixed here (internal loggers propagated upwards to the top)
        # We only want to assert a subset of the output
        assert "ROS1 pip pytemplate version 0.1.1" in out

    def test_default_noargs(self):
        pass

    def test_nargs(self):
        pass


# In case we run this directly, use pytest
if __name__ == '__main__':
    pytest.main(['-x', __file__])
