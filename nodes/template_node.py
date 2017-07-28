#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# License: Unspecified
#

from __future__ import absolute_import, division, print_function

"""
Module gathering all ROS side-effects
"""


##############################################################################
# Imports
##############################################################################
import argparse
import os
import sys
import time
from urlparse import urlsplit

import rospy
import ros1_template.msg as ros1_template_msgs
import ros1_template.srv as ros1_template_srvs


##############################################################################
# Main
##############################################################################

def show_description():
    return "ros template client test script"


def show_usage(cmd):
    cmd = cmd or sys.argv[0]
    return "{0} <origin_url> <replica_url> [--async]".format(cmd)


def show_epilog():
    return "never enough testing"


if __name__ == '__main__':

    # Note : since all the interface is made using ROS Services (XMLRPC), we do not need a ros node here

    args = rospy.myargv(sys.argv)
    parser = argparse.ArgumentParser(description=show_description(),
                                     usage=show_usage(args[0]),
                                     epilog=show_epilog(),
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # TODO : if no destination, assume .
    parser.add_argument('origin')
    parser.add_argument('replica')  # , default='.')  # TODO : default if not replica is to download (might conflict with local path already set in node at launch...)
    parser.add_argument('-a', '--async', dest='async', action='store_true', default=False,
                        help='async mode: use the asynchronous service API to avoid blocking, instead of the simple synchronous API')

    parsed_args = parser.parse_args(args[1:])

    if parsed_args.async:
        rospy.wait_for_service('concert_filesync/sync_async', timeout=5)
        try:
            sync_srv = rospy.ServiceProxy('concert_filesync/sync_async', gopher_filesync_srvs.FileSyncAS)
            check_srv = rospy.ServiceProxy('concert_filesync/sync_check', gopher_filesync_srvs.SyncCheck)

            request = gopher_filesync_srvs.FileSyncASRequest()
            request.pushlist = []
            request.pulllist = []

            from_url = parsed_args.origin
            to_url = parsed_args.replica
            if urlsplit(from_url).scheme and not urlsplit(to_url).scheme:  # from remote to local
                if urlsplit(from_url).scheme == 'ftp':
                    request.pulllist.append(
                        gopher_filesync_msgs.Pull(remote_origin=from_url, local_replica=to_url)
                    )
                else:
                    print("Unknown url scheme {0}".format(urlsplit(from_url).scheme))
                    sys.exit(127)
            elif urlsplit(to_url).scheme:
                if urlsplit(to_url).scheme == 'ftp':
                    request.pushlist.append(
                        gopher_filesync_msgs.Push(local_origin=from_url, remote_replica=to_url)
                    )
                else:
                    print("Unknown url scheme {0}".format(urlsplit(from_url).scheme))
                    sys.exit(127)
            else:
                print("Remote to Remote transfer currently not supported over ROS")
                sys.exit(127)

            resp = sync_srv(request)

            # watching check service and outputting to terminal until completion
            check_resp = gopher_filesync_srvs.SyncCheckResponse()
            check_resp.inprogress = resp.inprogress  # initial value from the sync response
            check_request = gopher_filesync_srvs.SyncCheckRequest()
            check_request.checklist = [t.id for t in resp.inprogress]  # the list of ids we receive from the service response is enough to request a check on these transfers
            while [p for p in check_resp.inprogress if p.progress < 100]:  # we want the list of transfer not finished yet
                check_resp = check_srv(check_request)
                print(check_resp)
                rospy.rostime.wallsleep(1)

        except rospy.ServiceException as e:
            print("Service call failed: {0}".format(e))

    else:

        rospy.wait_for_service('concert_filesync/sync', timeout=5)
        try:
            sync_srv = rospy.ServiceProxy('concert_filesync/sync', gopher_filesync_srvs.FileSyncS)

            request = gopher_filesync_srvs.FileSyncSRequest()
            request.pushlist = []
            request.pulllist = []

            from_url = parsed_args.origin
            to_url = parsed_args.replica
            if urlsplit(from_url).scheme and not urlsplit(to_url).scheme:  # from remote to local
                if urlsplit(from_url).scheme == 'ftp':
                    request.pulllist.append(
                        gopher_filesync_msgs.Pull(remote_origin=from_url, local_replica=to_url)
                    )
                else:
                    print("Unknown url scheme {0}".format(urlsplit(from_url).scheme))
                    sys.exit(127)
            elif urlsplit(to_url).scheme:
                if urlsplit(to_url).scheme == 'ftp':
                    request.pushlist.append(
                        gopher_filesync_msgs.Push(local_origin=from_url, remote_replica=to_url)
                    )
                else:
                    print("Unknown url scheme {0}".format(urlsplit(to_url).scheme))
                    sys.exit(127)
            else:
                print("Remote to Remote transfer currently not supported over ROS")
                sys.exit(127)

            resp = sync_srv(request)

        except rospy.ServiceException as e:
            print("Service call failed: {0}".format(e))
