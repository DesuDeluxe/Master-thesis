from simulation import Simulation
import random
from time import sleep

from multiprocessing import Process, Pipe

import logging
#logging.basicConfig(level=logging.DEBUG)
#logging.basicConfig(level=logging.INFO)
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)



STEP_REPEAT = 3#TODO

GENERATIONS = 2000
POPILATION_SIZE = 30
GAIT_STEPS = 4
ANGLES = 6
CHROMOSOME_SIZE = ANGLES*GAIT_STEPS
MAX_1=45
MIN_1=-45
MAX_2=180
MIN_2=-0
MAX_3=160
MIN_3=-90
MAX_MIN = [ [MIN_1,MAX_1], [MIN_2,MAX_2], [MIN_3,MAX_3] ]


LEG_GROUP1 = [1,4,5]#TODO
LEG_GROUP2 = [2,3,6]#TODO



s = 's'

stepss = [      [0,45,90,  0,90,90,  0,90,90,             0,45,90,  0,45,90, 0,90,90,],
                [60,45,90,  0,90,90,  0,90,90,             60,45,90,  60,45,90, 0,90,90,],
                [60,90,90,  0,90,90,  0,90,90,             60,90,90,  60,90,90, 0,90,90,],
                [0,90,90,  -60,90,90,  -60,90,90,             0,90,90,  0,90,90, -60,90,90,],
                [0,90,90,  0,45,90,  0,45,90,             0,90,90,  0,90,90, 0,45,90],
                [0,90,90,  60,45,90, 60,45,90,             0,90,90,  0,90,90, 60,45,90,],
                [0,90,90,  60,90,90,  60,90,90,             0,90,90,  0,90,90, 60,90,90,],
                [-60,90,90,  0,90,90,  0,90,90,             -60,90,90,  -60,90,90, 0,90,90,],
                ]
stepss = [[item for sublist in stepss for item in sublist],0]

'''
for sublist in l:
    for item in sublist:
        flat_list.append(item)
'''
'''
stepss = [      [0,45,90,  s,s,s,  s,s,s,             s,s,s,  s,s,s, s,s,s,
                60,45,90,s,s,s,  s,s,s,            s,s,s,  s,s,s, s,s,s,
                60,90,90,  s,s,s,  s,s,s,             s,s,s,  s,s,s, s,s,s,
                0,90,90,  s,s,s,  s,s,s,             s,s,s,  s,s,s, s,s,s,
                0,90,90,  s,s,s,  s,s,s,            s,s,s,  s,s,s, s,s,s,
                0,90,90,  s,s,s,  s,s,s,             s,s,s,  s,s,s, s,s,s,
                0,90,90,  s,s,s,  s,s,s,             s,s,s,  s,s,s, s,s,s,
                -60,90,90,  s,s,s,  s,s,s,             s,s,s,  s,s,s, s,s,s,]
                ]
'''

stepss = [

[30, 140, 65, 5, 60, 55, -40, 115, 60, 35, 115, 75, 35, 90, 75, -40, 115, 75, 35, 110, 65, -40, 110, 80],

]



def readFile(filename):
    gen = []
    go=0
    with open(filename) as file:
        for line in file:
            if "Generation: 499" in line:
                go=1
            line = line.strip() #preprocess line
            if len(line)>0 and go==1:
                if line[0] == "[":
                    gen.append(line[1:-2])
    print(gen)
    return gen

def convertToSim(chrom):
    chromNew = []
    for x in range(1, GAIT_STEPS+1):
        chromNew.append(chrom[0+(ANGLES*(x-1)):ANGLES*x])
    return chromNew

def convertToGen(chrom):
    c = []
    for x in range(0, GAIT_STEPS):
        c+=chrom[x]
    return c

def multi(obj):
    proc = Process(target=obj.setup)
    proc.start()
    return proc
def stepRepeat(x):
    z=[]
    for y in range(STEP_REPEAT):
        z+=x
    return z


def copytoLegs(ang):
    logger.debug('CPY IN: %s', str(ang))
    angles = []
    for y in range(GAIT_STEPS):
        angl = []
        for x in range(1, 7):
            if x in LEG_GROUP1:
                angl += ang[y][:int(ANGLES/2)]
            elif x in LEG_GROUP2:
                angl += ang[y][int(ANGLES/2):]
        angles.append(angl)
        logger.debug('COPYLEGS internal SIZE: %s', len(angl))
        logger.debug('COPYL internal EGS: %s', str(angl))

    logger.debug('COPYLEGS out SIZE: %s', len(angles))
    logger.debug('COPYLEGSout: %s', str(angles))
    return angles



def main():

    parent_conn, child_conn = Pipe()
    #sim.reset()
    #sim.setup()
    sim_proc = Process(target=Simulation, args=(child_conn,GAIT_STEPS*STEP_REPEAT,1111, 'gui'))
    sim_proc.start()
    #joints_seg = parent_conn.recv()
    #print(joints_seg)
    #simulation.setup()
    #stepss = readFile(r'E:\projekty\spider\algorithm\gen\generations-07_18_2018_13_26.txt')
    sleep(2)
    while True:
        for step in stepss:
            print(len(step))
            y = convertToSim(step)
            logger.debug('converted: %s', str(y))
            y = copytoLegs(y)
            logger.debug('copied: %s', str(y))
            y = stepRepeat(y)
            logger.debug('sent: %s', str(y))
            parent_conn.send(y)
            while True:
                if parent_conn.poll():
                    rec = parent_conn.recv()
                    if rec == "simok":
                        logger.debug('simok')
                        break
                    else:
                        logger.debug('wrong data, received %s', str(rec))
                #else:
                    #logger.debug('waitinf for simok')

            while True:
                if parent_conn.poll():
                    rec= parent_conn.recv()[-1]
                    logger.debug('data received : size %s %s', len(rec), str(rec))
                    if len(rec)==4:
                        contact, base_pos, base_angle, diff = rec
                    #if len(contact) == 6 and len(base_pos) == 3 and len(base_angle) == 3:
                        parent_conn.send('dataok')
                        logger.debug('dataok')
                        break
                else:
                    logger.debug('waitinf for correct data')

            parent_conn.send('reset')
            while True:
                #print('in loop3')
                if parent_conn.poll():
                    rec = parent_conn.recv()
                    if rec == "resok":
                        logger.debug('resok')
                        break
                #else:
                #    logger.debug('waitinf for reset confirm')
        #parent_conn.send('reset')


    print("end")


if __name__ == '__main__':
    main()
