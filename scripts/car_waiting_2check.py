
from __future__ import absolute_import
from __future__ import print_function

import os, yaml
import sys
import optparse
import random
import xml.etree.ElementTree as ET
import numpy as np
import pandas as pd
import copy,math
import tensorflow as tf
# -*- coding: utf-8 -*-

from sumolib import checkBinary  # noqa
import traci  # noqa


# you have to change to the directory where files exists.
os.chdir('/home/park/AI_traffic')

# read configuration
with open('yaml/sumo_configuration.yml') as f:
    conf = yaml.load(f,Loader=yaml.FullLoader)

global ph,doc, phase, root, step,step2,data_w_value2
global data1, data2, data3, data4, data5, data6, data7, data8, M_input , M_output
global option 

ph = [0, 0, 0, 0]
step = 0 
doc = ET.parse('f_road.net.xml')
phase = doc.find(".//phase")
root = doc.getroot()
i = 0
for phase in root.iter('phase'):
    i+=1
    random.seed(None)
    if i%2!=0:
        phase.attrib['duration'] = str(random.randint(20, 50))
        if i == 7:
            phase.attrib['duration'] = str(25)
        ph[i//2] = phase.attrib['duration']
doc.write('f_road.net.xml') 

ph2 = copy.deepcopy(ph)

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")



# load model
model = tf.keras.models.load_model('model/traffic_best.h5', compile=False)

option = 'ENM'

def generate_routefile():
    global car_north,car_south,car_east,car_west
    car_north=0
    car_south=0
    car_east=0
    car_west=0
    random.seed(None)  # make tests reproducible 
    N = 2000 # number of time steps
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
    
    ## SUMO simulator configuration 
    with open("f_road.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="car" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" \
        guiShape="passenger"/>

        <route id="12" edges="e1 e2" />  <route id="13" edges="e1 -e3" />  <route id="14" edges="e1 e4" />
        
        <route id="21" edges="-e2 -e1" />  <route id="23" edges="-e2 -e3" />  <route id="24" edges="-e2 e4" />
        
        <route id="31" edges="e3 -e1" />  <route id="32" edges="e3 e2" />  <route id="34" edges="e3 e4" />
        
        <route id="41" edges="-e4 -e1" />  <route id="42" edges="-e4 e2" />  <route id="43" edges="-e4 -e3" />
        """, file=routes)
        
        #  Total number of vehicles during simulation
        global vehNr
        vehNr = 0
        
        # Vehicle generated code in xml file
        # Changing the probability can change the number of vehicles created in each lane
        for i in range(N):
            if random.uniform(0, 1) < p12:
                print('    <vehicle id="12_%i" type="car" route="12" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                car_north += 1
            if random.uniform(0, 1) < p13:
                print('    <vehicle id="13_%i" type="car" route="13" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                car_north += 1
            if random.uniform(0, 1) < p14:
                print('    <vehicle id="14_%i" type="car" route="14" depart="%i" color="1,1,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
                car_north += 1
            if random.uniform(0, 1) < p21:
                print('    <vehicle id="21_%i" type="car" route="21" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
                car_south += 1
            if random.uniform(0, 1) < p23:
                print('    <vehicle id="23_%i" type="car" route="23" depart="%i" color="1,1,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
                car_south += 1
            if random.uniform(0, 1) < p24:
                print('    <vehicle id="24_%i" type="car" route="24" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
                car_south += 1
            if random.uniform(0, 1) < p31:
                print('    <vehicle id="31_%i" type="car" route="31" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                car_west += 1
            if random.uniform(0, 1) < p32:
                print('    <vehicle id="32_%i" type="car" route="32" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                car_west += 1
            if random.uniform(0, 1) < p34:
                print('    <vehicle id="34_%i" type="car" route="34" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                car_west += 1
            if random.uniform(0, 1) < p41:
                print('    <vehicle id="41_%i" type="car" route="41" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                car_east += 1
            if random.uniform(0, 1) < p42:
                print('    <vehicle id="42_%i" type="car" route="42" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                car_east += 1
            if random.uniform(0, 1) < p43:
                print('    <vehicle id="43_%i" type="car" route="43" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                car_east += 1
        
        # Number of vehicles created
        print("</routes>", file=routes)
        print("from north :",car_north)
        print("from south :",car_south)
        print("from east :",car_east)
        print("from west :",car_west)
       
def run():
    """execute the TraCI control loop"""
    global allwt,s_time
    print('ph : ',ph)
    allwt = [-1 for i in range(vehNr)] # Signal waiting time of generated vehicles
    print ( " allwt len : ",len(allwt))
    yphase = int(conf['phase']['yellow'])  # yellow signal time phase
    step = 0    # 
    step2 = 0   # sumo traffic light change counter
    s_time = 0  # signal time count

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
    
    global ph2,data_w_value2
    data_w_value2 = np.array([0,0,0,0,0,0,0,0])
    c_time = 0
    # we start with phase 2 where EW has green
    while traci.simulation.getMinExpectedNumber() > 0:
        
        
        traci.simulationStep()
        step += 1
        step2 += 1
        s_time += 1
        
        # List of current road vehicles
        vehicles=traci.vehicle.getIDList()
        
        cnt = [[0, 0], [0, 0], [0, 0], [0, 0]]
        
        # M_output 모델 결과
        
        if step2+2 == int(ph2[0])+3 or step2+2 == int(ph2[1])+int(ph2[0])+6 or step2+2 == int(ph2[2])+int(ph2[1])+int(ph2[0])+9 or step2+2 == int(ph2[3])+int(ph2[2])+int(ph2[1])+int(ph2[0])+12:
            data1c = [ficnt[0][1]]  
            data1w = [hwt[0][1]]  
            data2c = [ficnt[0][0]] 
            data2w = [hwt[0][0]]   
            data3c = [ficnt[1][1]] 
            data3w = [hwt[1][1]]   
            data4c = [ficnt[1][0]]  
            data4w = [hwt[1][0]]
            data5c = [ficnt[2][1]]  
            data5w = [hwt[2][1]]
            data6c = [ficnt[2][0]]
            data6w = [hwt[2][0]]
            data7c = [ficnt[3][1]] 
            data7w = [hwt[3][1]]
            data8c = [ficnt[3][0]]
            data8w = [hwt[3][0]]
            data1c2 = data1c2+[ficnt[0][1]]
            data1w2 = data1w2+[hwt[0][1]]
            data2c2 = data2c2+[ficnt[0][0]]
            data2w2 = data2w2+[hwt[0][0]]
            data3c2 = data3c2+[ficnt[1][1]]
            data3w2 = data3w2+[hwt[1][1]]
            data4c2 = data4c2+[ficnt[1][0]]
            data4w2 = data4w2+[hwt[1][0]]
            data5c2 = data5c2+[ficnt[2][1]]
            data5w2 = data5w2+[hwt[2][1]]
            data6c2 = data6c2+[ficnt[2][0]]
            data6w2 = data6w2+[hwt[2][0]]
            data7c2 = data7c2+[ficnt[3][1]]
            data7w2 = data7w2+[hwt[3][1]]
            data8c2 = data8c2+[ficnt[3][0]]
            data8w2 = data8w2+[hwt[3][0]]
            

            data_w = np.array([data1w[0], data2w[0], data3w[0], data4w[0], data5w[0], data6w[0], data7w[0], data8w[0]])
            data_w_value = np.array([0,0,0,0,0,0,0,0])
            
            # data_w_value : Number of vehicles over 100 seconds
            # data_w : Maximum waiting time for vehicles on each road
            
            for i in range(8):
                if data_w[i] > int(conf['sc']):
                    data_w_value[i] = 1
                else:
                    data_w_value[i] = 0
            data_w_value2 = np.vstack((data_w_value2,data_w_value))
            M_input = np.array([data1c[0], data_w_value[0], data2c[0], data_w_value[1], data3c[0], data_w_value[2], data4c[0], data_w_value[3], data5c[0], data_w_value[4], data6c[0], data_w_value[5], data7c[0], data_w_value[6], data8c[0], data_w_value[7]]).astype(np.float32)

            M_input = M_input.squeeze()
            M_input = np.expand_dims(M_input, axis=0)
        
            M_output = model.predict(M_input)
        
        if step2 < int(ph2[0]):
            traci.trafficlight.setPhase("gneJ4",0)
        elif step2 < int(ph2[0])+yphase:
            traci.trafficlight.setPhase("gneJ4",1)
        elif step2 < int(ph2[0])+int(ph2[1])+yphase:
            traci.trafficlight.setPhase("gneJ4",2)
        elif step2 < int(ph2[0])+int(ph2[1])+yphase*2:
            if step2%3==1:
                c_time += 1
                traci.trafficlight.setPhase("gneJ4",3)
                doc = ET.parse('f_road.net.xml')
                phase = doc.find(".//phase")
                root = doc.getroot()
                i = 0
                for phase in root.iter('phase'):
                    i+=1
                    random.seed(None)
                    if int(i/5) !=0:
                        if i%2!=0:
                         phase.attrib['duration'] = str(int(M_output[0][i//2]))
                         ph[i//2] = phase.attrib['duration']
                doc.write('f_road.net.xml')
                print('EW signal ', c_time,': ',ph[2:])
                ph2 = copy.deepcopy(ph)
        elif step2 < int(ph2[0])+int(ph2[1])+int(ph2[2])+yphase*2:
            traci.trafficlight.setPhase("gneJ4",4)
        elif step2 < int(ph2[0])+int(ph2[1])+int(ph2[2])+yphase*3:
            traci.trafficlight.setPhase("gneJ4",5)
        elif step2 < int(ph2[0])+int(ph2[1])+int(ph2[2])+int(ph2[3])+yphase*3:
            traci.trafficlight.setPhase("gneJ4",6)
        elif step2 < int(ph2[0])+int(ph2[1])+int(ph2[2])+int(ph2[3])+yphase*4:
            traci.trafficlight.setPhase("gneJ4",7)
        else:
            step2 = 0
            doc = ET.parse('f_road.net.xml')
            phase = doc.find(".//phase")
            root = doc.getroot()
            i = 0
            for phase in root.iter('phase'):
                i+=1
                random.seed(None)
                if int(i/5) ==0:     
                    if i%2!=0:
                        phase.attrib['duration'] = str(int(M_output[0][i//2]))
                        ph[i//2] = phase.attrib['duration']
            doc.write('f_road.net.xml')
            # print('M_output',M_output)
            print('NS signal ', c_time ,': ',ph[:2])
            ph2 = copy.deepcopy(ph)
        # print(vehicles)
        ficnt = [[0, 0], [0, 0], [0, 0], [0, 0]]
        hwt = [[0, 0], [0, 0], [0, 0], [0, 0]]

        for i in range(0,len(vehicles)): 
            
            car_waitingTime = int(traci.vehicle.getWaitingTime(vehicles[i]))
            car_Speed = traci.vehicle.getSpeed(vehicles[i])
            car_ID = traci.vehicle.getRoadID(vehicles[i])
            car_number = int(vehicles[i].split('_')[1])
            
                
                
            if car_waitingTime != 0:
                if car_ID == 'e1':
                    # car_north += 1
                    if vehicles[i].split('_')[0] == '13':
                        state[0][1] = 0                    
                        stop[0][1] = vehicles[i]
                        if car_Speed == 0:
                            cnt[0][1]+=1
                        if hwt[0][1] < allwt[car_number]:
                            hwt[0][1] = allwt[car_number]   
                    else:
                        state[0][0] = 0           
                        stop[0][0] = vehicles[i]
                        if car_Speed == 0:
                            cnt[0][0]+=1
                        if hwt[0][0] < allwt[car_number]:
                            hwt[0][0] = allwt[car_number]
                elif car_ID == '-e2':
                    # car_south +=1
                    if vehicles[i].split('_')[0] == '24':
                        state[1][1] = 0
                        stop[1][1] = vehicles[i]
                        if car_Speed == 0:
                            cnt[1][1]+=1
                        if hwt[1][1] < allwt[car_number]:
                            hwt[1][1] = allwt[car_number]
                    else:
                        state[1][0] = 0
                        stop[1][0] = vehicles[i]
                        if car_Speed == 0:
                            cnt[1][0]+=1
                        if hwt[1][0] < allwt[car_number]:
                            hwt[1][0] = allwt[car_number]
                elif car_ID == 'e3':
                    # car_west += 1
                    if vehicles[i].split('_')[0] == '32':
                        state[2][1] = 0
                        stop[2][1] = vehicles[i]
                        if car_Speed == 0:
                            cnt[2][1]+=1
                        if hwt[2][1] < allwt[car_number]:
                            hwt[2][1] = allwt[car_number]
                    else:
                        state[2][0] = 0
                        stop[2][0] = vehicles[i]
                        if car_Speed == 0:
                            cnt[2][0]+=1
                        if hwt[2][0] < allwt[car_number]:
                            hwt[2][0] = allwt[car_number]
                            
                elif car_ID == '-e4':
                    # car_east += 1
                    if vehicles[i].split('_')[0] == '41':
                        state[3][1] = 0
                        stop[3][1] = vehicles[i]
                        if car_Speed == 0:
                            cnt[3][1]+=1
                        if hwt[3][1] < allwt[car_number]:
                            hwt[3][1] = allwt[car_number]
                    else:
                        state[3][0] = 0
                        stop[3][0] = vehicles[i]
                        if car_Speed == 0:
                            cnt[3][0]+=1
                        if hwt[3][0] < allwt[car_number]:
                            hwt[3][0] = allwt[car_number]
        
            
            ficnt = cnt.copy()

            if stop[0][1] == vehicles[i]:
                if car_waitingTime == 0:
                    cnt[0][1] = 0
                    stop[0][1] = 0
                    state[0][1] = 1

            if stop[0][0] == vehicles[i]:
                if car_waitingTime == 0:
                    cnt[0][0] = 0
                    stop[0][0] = 0
                    state[0][0] = 1

            if stop[1][1] == vehicles[i]:
                if car_waitingTime == 0:
                    cnt[1][1] = 0
                    stop[1][1] = 0
                    state[1][1] = 1

            if stop[1][0] == vehicles[i]:
                if car_waitingTime == 0:
                    cnt[1][0] = 0
                    stop[1][0] = 0
                    state[1][0] = 1

            if stop[2][1] == vehicles[i]:
                if car_waitingTime == 0:
                    cnt[2][1] = 0
                    stop[2][1] = 0
                    state[2][1] = 1

            if stop[2][0] == vehicles[i]:
                if car_waitingTime == 0:
                    cnt[2][0] = 0
                    stop[2][0] = 0
                    state[2][0] = 1

            if stop[3][1] == vehicles[i]:
                if car_waitingTime == 0:   
                    cnt[3][1] = 0
                    stop[3][1] = 0
                    state[3][1] = 1

            if stop[3][0] == vehicles[i]:
                if car_waitingTime == 0:
                    cnt[3][0] = 0
                    stop[3][0] = 0
                    state[3][0] = 1

            
            if int(math.ceil(car_Speed)) == 0:
                allwt[car_number] +=1

        #print(cnt)  # each road [Number of Waiting car, Is it longer than 100 seconds?]

    data1 = np.array([data1c2, data1w2])
    data2 = np.array([data2c2, data2w2])
    data3 = np.array([data3c2, data3w2])
    data4 = np.array([data4c2, data4w2])
    data5 = np.array([data5c2, data5w2])
    data6 = np.array([data6c2, data6w2])
    data7 = np.array([data7c2, data7w2])
    data8 = np.array([data8c2, data8w2])    
    data_w_value2 = data_w_value2[1:,:]
    data_w_value2 = data_w_value2.T
    print(s_time)
    traci.close()
    sys.stdout.flush()
    

def datawrite():
    df = pd.read_csv('data_set/model_output.csv')
    df = df.T[1:]
    df = df.T
    # df = pd.DataFrame(columns=(['e1leftcar','e1leftwaiting','e1leftvalue','e1rightcar','e1rightwaiting','e1rightvalue','e2leftcar','e2leftwaiting','e2leftvalue','e2rightcar','e2rightwaiting','e2rightvalue','e3leftcar','e3leftwaiting','e3leftvalue','e3rightcar','e3rightwaiting','e3rightvalue','e4leftcar','e4leftwaiting','e4leftvalue','e4rightcar','e4rightwaiting','e4rightvalue','1_time','2_time','3_time','4_time']))
    # ,data_w_value[0]
    
    a = len(df)
    global fidata     
    global fitata         
    fidata = np.array([data1[0], data1[1],data_w_value2[0,:], data2[0], data2[1],data_w_value2[1,:], data3[0], data3[1],data_w_value2[2,:], data4[0], data4[1],data_w_value2[3,:], data5[0], data5[1],data_w_value2[4,:], data6[0], data6[1],data_w_value2[5,:], data7[0], data7[1],data_w_value2[6,:], data8[0], data8[1],data_w_value2[7,:]])
    fitata = fidata.T
    fitata = fitata[1:,:]
    # b = list( map(int, ph))
    for i in range(0,len(fitata)):
        df.loc[a+i] =  fitata[i].tolist() 
    
    df.to_csv("data_set/model_output.csv")



def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


def save_best():
    df = pd.read_csv('result/model_best_data.csv')
    # df = pd.DataFrame(columns=(['running_time','number_of_cars','north_car','south_car','east_car','west_car','30 upper','50 upper','80 upper','100 upper','150 upper','type']))
    df = df.T[1:]
    df = df.T
    a = len(df)
    data = np.array([s_time,len(allwt),car_north,car_south,car_east,car_west,car_30,car_50,car_80,car_100,car_150])
    data=data.T
    data = data.tolist() 
    data += [option]
    df.loc[a+1] =  data
    
    df.to_csv("result/model_best_data.csv")
    
    
def save_4025():
    df = pd.read_csv('result/model_4025_data.csv')
    # df = pd.DataFrame(columns=(['running_time','number_of_cars','north_car','south_car','east_car','west_car','30 upper','50 upper','80 upper','100 upper','150 upper','option']))
    df = df.T[1:]
    df = df.T
    a = len(df)
    data = np.array([s_time,len(allwt),car_north,car_south,car_east,car_west,car_30,car_50,car_80,car_100,car_150])
    data=data.T
    data = data.tolist() 
    data += [option]
    df.loc[a+1] =  data
    
    df.to_csv("result/model_4025_data.csv")
    
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

    run()

    # datawrite()
    
    car_30 =0     # Number of vehicles waiting more than 30 seconds
    car_50 =0     # Number of vehicles waiting more than 50 seconds
    car_80=0      # Number of vehicles waiting more than 80 seconds
    car_100 = 0   # Number of vehicles waiting more than 100 seconds
    car_150=0     # Number of vehicles waiting more than 150 seconds
    for i in range(len(allwt)):
        if allwt[i]>=100:
            car_100 += 1 
        if allwt[i]>=30:
            car_30 += 1 
        if allwt[i]>=50:
            car_50 += 1 
        if allwt[i]>=80:
            car_80 += 1 
        if allwt[i]>=150:
            car_150 += 1
    
    save_best()
    # save_4025()