#!/usr/bin/env python

#from __future__ import with_statement, division
#PKG = 'pr2_camera_self_filter'
#import roslib; roslib.load_manifest(PKG)

#import rospy
import subprocess

def run_xinit():
    subprocess.call(['sudo xinit'], shell=True)



if __name__ == '__main__':
    run_xinit()