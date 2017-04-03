from kalman import *
#from Kalman123 import *
import numpy as np
import time
import os, sys

def testKalman():
    state = np.array([[0],[0],[0],[0],[0],[0]])
    posx = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    posy = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    v = 1
    omega = 0
    dt = 1
    kf = Kalman()
    i = 0
    while i <= 9:
#        print (Kalman.update(kf, state, v, omega, dt, posx[i], posy[i]))
        state = Kalman.update(kf, state, v, omega, dt, [posx[i], posy[i]])
#        state = Kalman.correct(kf, state, [posx[i], posy[i]], v, omega, dt)
        i = i + 1;
        print (state)
        time.sleep(1)



if __name__== '__main__':
    testKalman()
