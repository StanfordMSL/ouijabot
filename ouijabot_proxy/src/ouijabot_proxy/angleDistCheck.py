#! bin/bash/python
import warnings as warn
import os, sys, time 
import multiprocessing as mp

#math
import numpy as np
import numpy.linalg as la
import numpy.random as rand
import cvxpy as cvx

#plot
import matplotlib.pyplot as plt

from ouijabotProxy import OuijabotProxy

class OuijaboyProxyTester(OuijabotProxy):
    """docstring for OuijaboyProxyTester"OuijaBotProxy def __init__(self, arg):
    """
    def __init__(self):
        pi=np.pi
        angles= [(pi, pi),
                (pi/2, pi), 
                (pi, pi/2),
                (3./2*pi, -pi/2),
                (-pi/2, 3./2*pi),
                (pi+.2, pi-.2),
                (pi-.2, pi+.2),
                (.2, -.2),
                (-.2, .2),
                (1.2*pi, 1.1*pi),
                (.3*pi, -.2*pi)]

        for a, b in angles:
            print self.circularDist(a,b)
        
        



if __name__ == '__main__':
    tester= OuijaboyProxyTester()
