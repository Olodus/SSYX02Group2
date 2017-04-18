#!/usr/bin/env python
import sys
import rospy
import csv
import time
import numpy as np
from rospy.numpy_msg import numpy_msg
from robotclient.msg import Floats
from robotclient.srv import *


def save_coordinates(n,rid):
    """
    :return: if not end node, measures position with UWB-radios and returns it. If end, returns current position.
    """
    print "first"
    rospy.init_node('position_sample_node')
    print "second"
    srv = 'get_coord' + str(rid)
    print 'third'
    rospy.wait_for_service(srv)
    print 'fourth'
    get_coords = rospy.ServiceProxy(srv, GetCoord)
    print 'fifth'
    xarr = np.array([], dtype=np.float32)
    yarr = np.array([], dtype=np.float32)
    i = 0
    error_count = 0
    print 'sixth'
    while i < n and error_count < 100:
        print("remaining time: " + str((n - i) * 0.14/60))
        print("Take:" + str(i))
        tmp_pos = np.array([],
                         dtype=np.float32)
        try:
            #startTime = time.time()
            f = Floats()
            f = get_coords(1)
            #stopTime = time.time()
            tmp_pos = f.data.data
            if np.size(tmp_pos) != 3:
                xarr = np.append(xarr, tmp_pos[0])
                yarr = np.append(yarr, tmp_pos[1])
                i += 1
            else:
                error_count += 1
                print("Invalid reading, check if all the unicorns are at home")
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        #print(str(tmp_pos))

    xmean = np.mean(xarr)
    ymean = np.mean(yarr)
    cov = np.cov(xarr,yarr)
    covx = np.cov(xarr)
    covy = np.cov(yarr)
    '''
    with open("/home/erikhigbie/TheTitans/robot_ws/src/robotclient/src/measurements"+str(filenumber)+".csv", "wb") as f:
        writer = csv.writer(f)
        writer.writerow(xarr)
        writer.writerow(yarr)
        writer.writerow([xmean, ymean])
        writer.writerow(cov[0,:])
        writer.writerow(cov[1,:])
    '''
    #pingTime= stopTime-startTime
    #print("Time" + str(pingTime))
    print("mean in y:" + str(ymean))
    print("mean in x:" + str(xmean))
    print("cov: " + str(cov))
    print("covx: " + str(covx))
    print("covy: " + str(covy))
    print("errors: " + str(error_count))
    return xmean


if __name__ == "__main__":
    print "yesterday..."
    try:
        runs = int (sys.argv[1])
        robot_id = int(sys.argv[2])
        print("start! Number of iterations: " + str(runs))
        save_coordinates(runs,rid)
    except rospy.ROSInterruptException:
        print("WTF???")
