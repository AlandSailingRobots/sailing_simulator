 
#!/usr/bin/python3

# Python packages
import socket
import sys
import select
import threading
import time
import copy
import atexit
from struct import *
import numpy as np
import queue

# Our packages
from network_voter_dbg import VoterNetwork

from matplotlib import pyplot as plt
import matplotlib.cm as cm

import json


if __name__ == '__main__':
    
    if len(sys.argv) >= 2:
        serverPort = int(sys.argv[1])
    else:
        serverPort = 58888
    
    net = VoterNetwork( "localhost", serverPort )

    running = 1

    bytes_received = 0
    data = bytearray()

    time.sleep(0.01)

    print("Start drawing thread")

    (course,veto,weight) = ([],[],0)

    # force square figure and square axes looks better for polar, IMO
    #fig = plt.figure(figsize=(8,8))
    fig = plt.figure()
    #ax = fig.add_axes([0.1, 0.1, 0.8, 0.8], polar=True)
    N = 360
    #Plot with 0 degree at North and counterclockwise
    for i in range(7): # number of plots expected
        ax = fig.add_subplot(3,3,i, projection='polar')
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)      
    #ax.plot(X,lines[li]*yScalingFactor,label=linelabels[li],color=color,linestyle=ls)
    name_to_index = {} # Dictionnary used to plot into the good subplot depending on voter name
    nb_of_voter = 0    
    name = ""
    print("Init done")

    while(running):

        try:
            while( net.connected() ):
                #Send garbage, so the server know there is a connection
                net.sendTrash()

                
                
                deb = time.time()

                data = net.receiveData()
                if len(data) > 0:
                    (course,veto,weight,name) = net.receiveVoterData(data)
                    print("Boolean debug output: ", any(veto), all(veto))
                    print("Course debug output: ", any(course), all(course))
                    print("Weight debug output: ", weight)
                    print("Name debug output: ", name)
                    
                    if (not name in name_to_index) and (name != ""): # add a key to the new voter if not already in the dict
                        name_to_index[name] = nb_of_voter
                        nb_of_voter += 1
                        ax = fig.add_subplot(nb_of_voter,3,name_to_index[name]+1, projection='polar')
                        #axn = plt.subplot(nb_of_voter,3,name_to_index[name]+1) # +1 because it starts at 1
                    
                else:
                    print("Data empty? length-> ", len(data))
                
                

                #Plot polar bar
                
                #ax = fig.add_axes([0.1, 0.1, 0.8, 0.8], polar=True)
                N = 360
                #Plot with 0 degree at North and counterclockwise
                #ax = fig.add_subplot(111, projection='polar')
                ax.set_theta_zero_location('N')
                ax.set_theta_direction(-1) 

                theta = np.arange(0.0, 2*np.pi, 2*np.pi/N)
                theta = [2*i*np.pi/N for i in range(N)]
                #radii = 10*np.random.rand(N)
                if name == "wind_voter":  # in the case of the wind voter, we're interested in its veto
                    radii = veto.copy()
                else:
                    radii = course.copy()
                
                #width = np.pi/4*np.random.rand(N)
                print(type(theta), " --- ", len(theta))
                print(type(radii), " --- ",len(radii))
                #import pdb; pdb.set_trace()

                if (len(radii) == 0):
                    radii = [np.random.randint(N)/N for i in range(N)]
                    print("Warning, radii is empty")
                bars = ax.bar(theta, radii, width=0.015, bottom=0.0)
                for r,bar in zip(radii, bars):
                    bar.set_facecolor( cm.jet(r/10.))
                    bar.set_alpha(0.5)

                plt.xticks(np.arange(0, 360.0*np.pi/180.0, 25.0*np.pi/180.0))
                if name == "wind_voter": # change scale as the veto is a value of 0 or 1
                    plt.ylim(0.0, 1.1)
                else:
                    plt.ylim(0.0, 105.0)

                plt.pause(0.1)
                plt.cla()

                #plt.show()
                print("Loop in")

        except socket.error as msg:
            print("Error :", msg)

        print("Loop out")

        #except:
         #   print(sys.exc_info())
            #pdb.pm()

    
    


    
