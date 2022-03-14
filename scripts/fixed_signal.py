from __future__ import absolute_import
from __future__ import print_function

import os, yaml
import sys
import optparse
import random,copy,math
import xml.etree.ElementTree as ET
import numpy as np
import pandas as pd
# import csv
global allwt
allwt = [[0,0]]
global ph, doc, phase, root, i

os.chdir('/home/park/AI_traffic')

# read configuration
with open('yaml/sumo_configuration.yml') as f:
    conf = yaml.load(f,Loader=yaml.FullLoader)


ph = [0, 0, 0, 0]
ph1 = [40 , 25 , 40 , 25]
doc = ET.parse('f_road.net.xml')
phase = doc.find(".//phase")
root = doc.getroot()
i = 0
for phase in root.iter('phase'):
    i+=1
    random.seed(None)
    if i%2!=0:
        phase.attrib['duration'] = str(ph1[int(i//2)])
        ph[i//2] = phase.attrib['duration']
doc.write('f_road.net.xml')
print('ph_now :',ph)

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa

def generate_routefile():
    random.seed(3)  # make tests reproducible 
    N = 2000  # number of time steps
    # demand per second from different directions
    p12 = float(conf['prob']['ns'])     # N to S
    p13 = float(conf['prob']['ne'])     # N to E
    p14 = float(conf['prob']['nw'])     # N to W
    p21 = float(conf['prob']['sn'])     # S to N
    p23 = float(conf['prob']['se'])     # S to E
    p24 = float(conf['prob']['sw'])     # S to W
    p31 = float(conf['prob']['en'])     # E to N
    p32 = float(conf['prob']['es'])     # E to S
    p34 = float(conf['prob']['ew'])     # E tp W
    p41 = float(conf['prob']['wn'])     # W to N
    p42 = float(conf['prob']['ws'])     # W to S
    p43 = float(conf['prob']['we'])     # W to E
    
    with open("f_road.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="car" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" \
        guiShape="passenger"/>

        <route id="12" edges="e1 e2" />  <route id="13" edges="e1 -e3" />  <route id="14" edges="e1 e4" />
        
        <route id="21" edges="-e2 -e1" />  <route id="23" edges="-e2 -e3" />  <route id="24" edges="-e2 e4" />
        
        <route id="31" edges="e3 -e1" />  <route id="32" edges="e3 e2" />  <route id="34" edges="e3 e4" />
        
        <route id="41" edges="-e4 -e1" />  <route id="42" edges="-e4 e2" />  <route id="43" edges="-e4 -e3" />
        """, file=routes)
        global vehNr
        vehNr = 0
        
        for i in range(N):
            if random.uniform(0, 1) < p12:
                print('    <vehicle id="12_%i" type="car" route="12" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p13:
                print('    <vehicle id="13_%i" type="car" route="13" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p14:
                print('    <vehicle id="14_%i" type="car" route="14" depart="%i" color="1,1,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p21:
                print('    <vehicle id="21_%i" type="car" route="21" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p23:
                print('    <vehicle id="23_%i" type="car" route="23" depart="%i" color="1,1,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p24:
                print('    <vehicle id="24_%i" type="car" route="24" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p31:
                print('    <vehicle id="31_%i" type="car" route="31" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p32:
                print('    <vehicle id="32_%i" type="car" route="32" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p34:
                print('    <vehicle id="34_%i" type="car" route="34" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p41:
                print('    <vehicle id="41_%i" type="car" route="41" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p42:
                print('    <vehicle id="42_%i" type="car" route="42" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < p43:
                print('    <vehicle id="43_%i" type="car" route="43" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
        print("</routes>", file=routes)


def run():
    """execute the TraCI control loop"""
    yphase = 3
    step = 1
    s_time = 0
    step2 = 0   
    ficnt = [[0, 0], [0, 0], [0, 0], [0, 0]]
    state = [[1, 1], [1, 1], [1, 1], [1, 1]]
    stop = [[0, 0], [0, 0], [0, 0], [0, 0]]
    hwt = [[0, 0], [0, 0], [0, 0], [0, 0]]
    global data1, data2, data3, data4, data5, data6, data7, data8, time1,time2,time3,time4,doc, phase, root
    data1c = []
    data2c = []
    data3c = []
    data4c = []
    data5c = []
    data6c = []
    data7c = []
    data8c = []
    data1w = []
    data2w = []
    data3w = []
    data4w = []
    data5w = []
    data6w = []
    data7w = []
    data8w = []
    time1 =[]
    time2 =[]
    time3 =[]
    time4 =[]
    # we start with phase 2 where EW has green
    while traci.simulation.getMinExpectedNumber() > 0:
        
        traci.simulationStep()
        step += 1
        step2 += 1
        s_time += 1
        vehicles=traci.vehicle.getIDList()
        cnt = [[0, 0], [0, 0], [0, 0], [0, 0]]
        
        

        
        
        # if step+2 == int(ph[0]) or step+2 == int(ph[1])+int(ph[0])+3 or step+2 == int(ph[2])+int(ph[1])+int(ph[0])+6 or step+2 == int(ph[3])+int(ph[2])+int(ph[1])+int(ph[0])+9:
        #     data1c = data1c+[ficnt[0][1]]
        #     data1w = data1w+[hwt[0][1]]
        #     data2c = data2c+[ficnt[0][0]]
        #     data2w = data2w+[hwt[0][0]]
        #     data3c = data3c+[ficnt[1][1]]
        #     data3w = data3w+[hwt[1][1]]
        #     data4c = data4c+[ficnt[1][0]]
        #     data4w = data4w+[hwt[1][0]]
        #     data5c = data5c+[ficnt[2][1]]
        #     data5w = data5w+[hwt[2][1]]
        #     data6c = data6c+[ficnt[2][0]]
        #     data6w = data6w+[hwt[2][0]]
        #     data7c = data7c+[ficnt[3][1]]
        #     data7w = data7w+[hwt[3][1]]
        #     data8c = data8c+[ficnt[3][0]]
        #     data8w = data8w+[hwt[3][0]]
        #     time1 = time1 + [int(ph[0])]
        #     time2 = time2 + [int(ph[1])]
        #     time3 = time3 + [int(ph[2])]
        #     time4 = time4 + [int(ph[3])]
        #     if step+2 == int(ph[3])+int(ph[2])+int(ph[1])+int(ph[0])+9:
        #         step = 0
                
            
        
        for i in range(0,len(vehicles)): 
                
            if traci.vehicle.getWaitingTime(vehicles[i]) != 0:
                if traci.vehicle.getRoadID(vehicles[i]) == 'e1':
                    if vehicles[i].split('_')[0] == '13':
                        state[0][1] = 0                    
                        stop[0][1] = vehicles[i]
                        if traci.vehicle.getSpeed(vehicles[i]) == 0:
                            cnt[0][1]+=1
                    else:
                        state[0][0] = 0           
                        stop[0][0] = vehicles[i]
                        if traci.vehicle.getSpeed(vehicles[i]) == 0:
                            cnt[0][0]+=1
                elif traci.vehicle.getRoadID(vehicles[i]) == '-e2':
                    if vehicles[i].split('_')[0] == '24':
                        state[1][1] = 0
                        stop[1][1] = vehicles[i]
                        if traci.vehicle.getSpeed(vehicles[i]) == 0:
                            cnt[1][1]+=1
                    else:
                        state[1][0] = 0
                        stop[1][0] = vehicles[i]
                        if traci.vehicle.getSpeed(vehicles[i]) == 0:
                            cnt[1][0]+=1
                elif traci.vehicle.getRoadID(vehicles[i]) == 'e3':
                    if vehicles[i].split('_')[0] == '32':
                        state[2][1] = 0
                        stop[2][1] = vehicles[i]
                        if traci.vehicle.getSpeed(vehicles[i]) == 0:
                            cnt[2][1]+=1
                    else:
                        state[2][0] = 0
                        stop[2][0] = vehicles[i]
                        if traci.vehicle.getSpeed(vehicles[i]) == 0:
                            cnt[2][0]+=1
                elif traci.vehicle.getRoadID(vehicles[i]) == '-e4':
                    if vehicles[i].split('_')[0] == '41':
                        state[3][1] = 0
                        stop[3][1] = vehicles[i]
                        if traci.vehicle.getSpeed(vehicles[i]) == 0:
                            cnt[3][1]+=1
                    else:
                        state[3][0] = 0
                        stop[3][0] = vehicles[i]
                        if traci.vehicle.getSpeed(vehicles[i]) == 0:
                            cnt[3][0]+=1
        
            
            ficnt = cnt.copy()

    print(s_time)
    traci.close()
    sys.stdout.flush()

def run2():
    """execute the TraCI control loop"""
    global allwt
    print('ph : ',ph)
    allwt = [-1 for i in range(vehNr)]
    print ( " allwt len : ",len(allwt))
    yphase = 3
    step = 0
    step2 = 0   
    s_time = 0

    ficnt = [[0, 0], [0, 0], [0, 0], [0, 0]]
    state = [[1, 1], [1, 1], [1, 1], [1, 1]]
    stop = [[0, 0], [0, 0], [0, 0], [0, 0]]
    hwt = [[0, 0], [0, 0], [0, 0], [0, 0]]
    global data1, data2, data3, data4, data5, data6, data7, data8,doc, phase, root, M_input , M_output
    data1c2 = []
    data2c2 = []
    data3c2 = []
    data4c2 = []
    data5c2 = []
    data6c2 = []
    data7c2 = []
    data8c2 = []
    data1w2 = []
    data2w2 = []
    data3w2 = []
    data4w2 = []
    data5w2 = []
    data6w2 = []
    data7w2 = []
    data8w2 = []
    
    data1 = []
    data2 = []
    data3 = []
    data4 = []
    data5 = []
    data6 = []
    data7 = []
    data8 = []
    
    data1c = []
    data2c = []
    data3c = []
    data4c = []
    data5c = []
    data6c = []
    data7c = []
    data8c = []
    data1w = []
    data2w = []
    data3w = []
    data4w = []
    data5w = []
    data6w = []
    data7w = []
    data8w = []
    # we start with phase 2 where EW has green
    while traci.simulation.getMinExpectedNumber() > 0:
        global ph2
        
        traci.simulationStep()
        step += 1
        step2 += 1
        s_time += 1
        vehicles=traci.vehicle.getIDList()
        cnt = [[0, 0], [0, 0], [0, 0], [0, 0]]
        
        for i in range(0,len(vehicles)): 
            
            
            car_waitingTime = int(traci.vehicle.getWaitingTime(vehicles[i]))
            car_Speed = traci.vehicle.getSpeed(vehicles[i])
            car_ID = traci.vehicle.getRoadID(vehicles[i])
            if car_waitingTime != 0:
                if car_ID == 'e1':
                    if vehicles[i].split('_')[0] == '13':
                        state[0][1] = 0                    
                        stop[0][1] = vehicles[i]
                        if car_Speed == 0:
                            cnt[0][1]+=1
                    else:
                        state[0][0] = 0           
                        stop[0][0] = vehicles[i]
                        if car_Speed == 0:
                            cnt[0][0]+=1
                elif car_ID == '-e2':
                    if vehicles[i].split('_')[0] == '24':
                        state[1][1] = 0
                        stop[1][1] = vehicles[i]
                        if car_Speed == 0:
                            cnt[1][1]+=1
                    else:
                        state[1][0] = 0
                        stop[1][0] = vehicles[i]
                        if car_Speed == 0:
                            cnt[1][0]+=1
                elif car_ID == 'e3':
                    if vehicles[i].split('_')[0] == '32':
                        state[2][1] = 0
                        stop[2][1] = vehicles[i]
                        if car_Speed == 0:
                            cnt[2][1]+=1
                    else:
                        state[2][0] = 0
                        stop[2][0] = vehicles[i]
                        if car_Speed == 0:
                            cnt[2][0]+=1
                elif car_ID == '-e4':
                    if vehicles[i].split('_')[0] == '41':
                        state[3][1] = 0
                        stop[3][1] = vehicles[i]
                        if car_Speed == 0:
                            cnt[3][1]+=1
                    else:
                        state[3][0] = 0
                        stop[3][0] = vehicles[i]
                        if car_Speed == 0:
                            cnt[3][0]+=1
        
            
            ficnt = cnt.copy()
                
            if stop[0][1] == vehicles[i]:
                if car_waitingTime == 0:
                    cnt[0][1] = 0
                    stop[0][1] = 0
                    state[0][1] = 1
                    hwt[0][1] = 0
                elif car_waitingTime > hwt[0][1]:
                    hwt[0][1] = car_waitingTime
            if stop[0][0] == vehicles[i]:
                if car_waitingTime == 0:
                    cnt[0][0] = 0
                    stop[0][0] = 0
                    state[0][0] = 1
                    hwt[0][0] = 0
                elif car_waitingTime > hwt[0][0]:
                    hwt[0][0] = car_waitingTime
            if stop[1][1] == vehicles[i]:
                if car_waitingTime == 0:
                    cnt[1][1] = 0
                    stop[1][1] = 0
                    state[1][1] = 1
                    hwt[1][1] = 0
                elif car_waitingTime > hwt[1][1]:
                    hwt[1][1] = car_waitingTime
            if stop[1][0] == vehicles[i]:
                if car_waitingTime == 0:
                    cnt[1][0] = 0
                    stop[1][0] = 0
                    state[1][0] = 1
                    hwt[1][0] = 0
                elif car_waitingTime > hwt[1][0]:
                    hwt[1][0] = car_waitingTime
            if stop[2][1] == vehicles[i]:
                if car_waitingTime == 0:
                    cnt[2][1] = 0
                    stop[2][1] = 0
                    state[2][1] = 1
                    hwt[2][1] = 0
                elif car_waitingTime > hwt[2][1]:
                    hwt[2][1] = car_waitingTime
            if stop[2][0] == vehicles[i]:
                if car_waitingTime == 0:
                    cnt[2][0] = 0
                    stop[2][0] = 0
                    state[2][0] = 1
                    hwt[2][0] = 0
                elif car_waitingTime > hwt[2][0]:
                    hwt[2][0] = car_waitingTime
            if stop[3][1] == vehicles[i]:
                if car_waitingTime == 0:   
                    cnt[3][1] = 0
                    stop[3][1] = 0
                    state[3][1] = 1
                    hwt[3][1] = 0
                elif car_waitingTime > hwt[3][1]:
                    hwt[3][1] = car_waitingTime
            if stop[3][0] == vehicles[i]:
                if car_waitingTime == 0:
                    cnt[3][0] = 0
                    stop[3][0] = 0
                    state[3][0] = 1
                    hwt[3][0] = 0
                elif car_waitingTime > hwt[3][0]:
                    hwt[3][0] = car_waitingTime
                        
            car_number = int(vehicles[i].split('_')[1])
            
            #print("Car_spped{} :".format(car_ID),car_Speed)
            if int(math.ceil(car_Speed)) == 0:
                allwt[car_number] +=1
        # print()     

        

        # print(allwt)
        # print(len(allwt))
    data1 = np.array([data1c2, data1w2])
    data2 = np.array([data2c2, data2w2])
    data3 = np.array([data3c2, data3w2])
    data4 = np.array([data4c2, data4w2])
    data5 = np.array([data5c2, data5w2])
    data6 = np.array([data6c2, data6w2])
    data7 = np.array([data7c2, data7w2])
    data8 = np.array([data8c2, data8w2])    
    print(s_time)
    traci.close()
    sys.stdout.flush()


    

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "f_road.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
    run2()
    
    # generate_routefile()
    
    car_s = 0
    for i in range(len(allwt)):
        if allwt[i]>=100:
            car_s += 1 
            
    print(car_s)