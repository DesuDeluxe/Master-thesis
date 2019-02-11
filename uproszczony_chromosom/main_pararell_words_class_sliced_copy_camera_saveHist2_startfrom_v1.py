from simulation import Simulation
from genetic import Genetic
import random
from time import sleep
from datetime import datetime
import os
import sys
import logging

import ast




#import matplotlib
#import matplotlib.pyplot as plt
#logging.basicConfig(level=logging.DEBUG)
#logging.basicConfig(level=logging.INFO)

logger = logging.getLogger(__name__)


from multiprocessing import Process, Pipe


PREVIOUS = 1

GENERATIONS = 5000
POPILATION_SIZE = 100#20
GAIT_STEPS = 4
ANGLES = 6#18 different for all joints
CHROMOSOME_SIZE = ANGLES*GAIT_STEPS
BEST_SAMPLES = 5#int(POPILATION_SIZE*0.2)
LUCKY_FEW = 5#int(BEST_SAMPLES*0.1)
MUTATION_CHANCE = 0.8
CROSSOVER_CHANCE = 0.8
TOURNAMENT = 5
if LUCKY_FEW <0:
    LUCKY_FEW = 0

NEWGENES = 0
RANDOM_MUTATE_POSITIONS = random.randint(0,ANGLES)

MAX_PARALLEL = 20
POP_STEPS = int(POPILATION_SIZE/MAX_PARALLEL)


STEP_REPEAT = 2#TODO
CHROM_TO_COPY = 1

MAX_1=40#50
MIN_1=-40#-50
MAX_2=150#180
MIN_2=30 #0
MAX_3=120#150#160
MIN_3=50#30#-90
MAX_MIN = [ [MIN_1,MAX_1], [MIN_2,MAX_2], [MIN_3,MAX_3] ]


LEG_GROUP1 = [1,4,5]#TODO
LEG_GROUP2 = [2,3,6]#TODO




FITNESS_MAX_HISTORY = [0]

#def readFromFile(filename):
def readFromFile(filename):
    previousGen = [];
    with open(filename, 'r') as f:
        for line in f:
            if(line[0]=='['):
                previousGen.append(ast.literal_eval(line[:-2]))

    gen = int(len(previousGen)/POPILATION_SIZE)
    del previousGen[:-POPILATION_SIZE]
    #print(len(previousGen))
    #print(previousGen[0])
    #print(previousGen)
    return previousGen, gen, filename

class Multi(object):
    objCount=0;
    def __init__(self):
        self.name=Multi.objCount;
        Multi.objCount+=1;
        self.simok = False
        self.data = []
        self.indi = []

    def multi_start(self):
        self.parent_pipe, child_pipe = Pipe()
        self.proc = Process(target=Simulation, args=(child_pipe, GAIT_STEPS*STEP_REPEAT,self.name,))
        self.proc.start()

    def multi_stop(self):
            self.proc.join()

def newFile(prevfile = None):
    path, filename = os.path.split(os.path.abspath(__file__))
    filename1 = path +r'/generations-' + datetime.now().strftime("%m_%d_%Y_%H_%M") + ".txt"
    file = open(filename1, 'w')
    if(prevfile is not None):
        file.write("previous file: %s\n" % (prevfile))

    file.write("POPILATION_SIZE: %s, GAIT_STEPS: %s, MUTATION_CHANCE: %s, CROSSOVER_CHANCE: %s, BEST_SAMPLES: %s, STEP_REPEAT: %s, TOURNAMENT: %s\n" % (POPILATION_SIZE, GAIT_STEPS,MUTATION_CHANCE,CROSSOVER_CHANCE, BEST_SAMPLES, STEP_REPEAT, TOURNAMENT))
    file.close()

    #history
    filename2 = path +r'HISTORY-' + datetime.now().strftime("%m_%d_%Y_%H_%M") + ".txt"
    file = open(filename2, 'w')
    #file.write("POPILATION_SIZE: %s, GAIT_STEPS: %s, MUTATION_CHANCE: %s, BEST_SAMPLES: %s, STEP_REPEAT: %s, TOURNAMENT: %s\n" % (POPILATION_SIZE, GAIT_STEPS,MUTATION_CHANCE, BEST_SAMPLES, STEP_REPEAT, TOURNAMENT))
    file.close()

    return filename1, filename2


def saveToFile(filename, gen, data):
    file = open(filename, 'a')
    file.write("%s,\n" % gen)
    for items in data:
        for item in items:
            file.write("%s,\n" % item)
    file.write("\n")
    file.close()


def convertToSim(chrom):#split chromosome to equal gait steps to send one after another
    #for x in range(1, GAIT_STEPS+1):
        #chromNew.append(chrom[0+(ANGLES*(x-1)):ANGLES*x])


    chromNew = []
    for x in range(0, GAIT_STEPS*ANGLES, ANGLES):
        #print ('x')
        #print (x)
        a = chrom[x:x+ANGLES]
        #print(a)
        chromNew.append(a)
    #print("conw")
    #print(chromNew)
    return chromNew

def convertToGen(chrom):
    c = []
    for x in range(0, GAIT_STEPS):
        c+=chrom[x]
    return c

def slicePop(pop):#for pararel computing
    sliced = []
    for x in range(0, POPILATION_SIZE, MAX_PARALLEL):
        #print ('x')
        #print (x)
        sliced.append(pop[x:x+MAX_PARALLEL])
    #print(sliced)
    return sliced

def stepRepeat2(x):
    z=[]
    for y in range(STEP_REPEAT):
        z+=x
    return z

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

def ensure_uniqness(d):
    while (1):
        b = d
        if (b not in d):
            return 0

def main():
    simu = []

    for x in range(MAX_PARALLEL):
        simu.append(Multi())
        simu[-1].multi_start()

    t=0
    while not (t == MAX_PARALLEL):
        for sim in simu:
            if sim.parent_pipe.poll():
                joints_seg = sim.parent_pipe.recv()
                if len(joints_seg)==6:
                    t+=1
    #simulation.setup()
#################### genetic
#fitness odleglosc od zrodka (x albo y) plus 10x za kazda noge na ziemi minus[kat podst - aktualny] - x3(xyx)

#10*sim_data[1][1] - (0 - abs(sim_data[2][1])) - (0 - abs(sim_data[2][2])) + [10*x for x in sim_data[0]]
    genetica = Genetic(POPILATION_SIZE, CHROMOSOME_SIZE, GAIT_STEPS, MAX_MIN, joints_seg, BEST_SAMPLES, LUCKY_FEW, TOURNAMENT, MUTATION_CHANCE, CROSSOVER_CHANCE)
#resetBasePositionAndOrientation
    #population = []
    gen = 0

    ##########################################prev
    population,gen, prevfile = readFromFile('/run/media/desu/n1/projekty/spider/algorithm/gen/simple/uproszczenie_kopiowanie/generations-02_05_2019_03_49.txt')
    temp = []
    for ind in population:
        temp.append(convertToSim(ind))
    population = temp
    del temp
    filename, history =newFile(prevfile)
    ##
    ###############################################new
    '''
    filename, history =newFile()
    population = genetica.gen_population()

    ##
    '''

    #print("population F{}".format(population))
    logger.debug('new pop: %s', str(population))

    #print(population)
    #population[0] = convertToSim(ppp[0])
    #population[1] = convertToSim(ppp[1])

    pop_data = []
    pop_fitness = [0,0]
    pop_sorted = [0,0]



    for generation in range(1,GENERATIONS+1):
        #FITNESS_MAX_HISTORY.append(pop_fitness[0])
        #show_plot(fig, li, generation)
        #print("Generation:", str(generation) + " previous 2 best fitness: ",str(pop_fitness[0]), str(pop_fitness[1]) )
        #print("Generation: {}, previous 2 best fitness: {}, {}, chromosomes: {}, {}".format(generation+gen,pop_fitness[0],pop_fitness[1], pop_sorted[0], pop_sorted[1]) )
        print("Generation:", str(generation+gen) + " previous 2 best fitness: ",str(pop_fitness[0]), str(pop_fitness[1]) )
        weighted_population = []

        slicedd = slicePop(population)
        #print(len(slicedd))

        logger.debug('check1 pop: %d', len(population))
        popCurr = 0
        for slic in slicedd:
            print("current populations", popCurr, "of",POPILATION_SIZE, end="\r")
            for idx, individual in enumerate(slic):
                simu[idx].indi = individual
                #print(individual)
                y = copytoLegs(individual)
                logger.debug('sent: %s', str(y))
                y = stepRepeat(y)
                logger.debug('repeated %s', y)
                logger.debug('%d', len(y))
                simu[idx].parent_pipe.send(y)

            t=0
            while not (t == MAX_PARALLEL):#if all processes sent data
                for sim in simu:
                    if sim.simok == False:
                        if sim.parent_pipe.poll():
                            rec = sim.parent_pipe.recv()
                            logger.debug('rec1 %s', str(rec))
                            if rec == "simok":
                                sim.simok = True
                                t+=1
            t=0
            while not (t == MAX_PARALLEL):
                for sim in simu:
                    if sim.parent_pipe.poll():
                        rec = sim.parent_pipe.recv()#[-1]#!!!!!!!!!!!!!!!!!tylko ostatnie biore? i potem zapisuje? co z polkrokami
                        logger.debug('rec2 %s', str(rec))
                        logger.debug('rec2 len %d', len(rec))
                        if len(rec)==GAIT_STEPS*STEP_REPEAT:#if len(contact) == 6 and len(base_pos) == 3 and len(base_angle) == 3:
                            #contact, base_pos, base_angle, diff = rec
                            sim.data = rec
                            sim.parent_pipe.send('dataok')
                            logger.debug('M dataok sent')
                            t+=1
            t=0

            for sim in simu:
                sim.simok = False
                sim.parent_pipe.send('reset')
                logger.debug('M reset sent')

            while not (t == MAX_PARALLEL):
                for sim in simu:
                    if sim.parent_pipe.poll():
                        rec = sim.parent_pipe.recv()
                        logger.debug('rec3 %s', str(rec))
                        if rec == "resok":
                            t+=1
            t=0

            for sim in simu:
                dd = 0
                for x in sim.data:
                    #print(len(sim.data))
                    if x == sim.data[-1]:
                        dd+=genetica.fitness(x)
                    else:
                        dd+=genetica.stability(x)
                fitness_val = dd

                individual = convertToGen(sim.indi)

                pair = (individual, fitness_val)
                weighted_population.append(pair)
            popCurr+=MAX_PARALLEL
        population = []

        logger.debug('check2')
        #for x in range(GAIT_STEPS)
        #pop_sorted, pop_fitness = genetica.sort_pop(weighted_population)
        pop_sorted,  pop_fitness = genetica.sort_pop(weighted_population)
        #pop_sort = [pop_sorted, pop_fitness]
        #print(pop_sorted[0])
        logger.debug('%s',str(pop_fitness))
        #saveToFile(str(generation)+" best fitness: " + str(pop_fitness[0]), [pop_sorted + pop_fitness])
        #saveToFile(filename, "Generation: " + str(generation)+ " best fitness: " + str(pop_fitness[0]), [pop_fitness[:15], pop_sorted[:15]])
        saveToFile(filename, "Generation: " + str(generation+gen)+ " best fitness: " + str(pop_fitness[0]), [pop_fitness, pop_sorted])
        saveToFile(history, str(generation+gen)+ "," + str(pop_fitness[0]), [])
        #print("saved to file")
        logger.debug('check3')

        #2 parents survive
        #print(pop_sorted[1])


        for x in range(CHROM_TO_COPY):
            a = convertToSim(pop_sorted[x])
            if (a not in population):
                population.append(a)
            for y in range(0, 5):
                while (1):
                    b = genetica.mutate(a,1,GAIT_STEPS)
                    if (b not in population):
                        population.append(b)
                        break
            for y in range(0, 5):
                while (1):
                    b = genetica.mutate(a,1,GAIT_STEPS, RANDOM_MUTATE_POSITIONS)
                    if (b not in population):
                        population.append(b)
                        break


        new_gen_population = genetica.gen_population()
        new_gen_population = random.sample(new_gen_population, NEWGENES)

        population += new_gen_population

        logger.debug('check4')
        logger.debug('INITIAL population: %s', str(population))

        old_sel = pop_sorted

        pop = [x[0] for x in weighted_population] #population
        b = [x[1] for x in weighted_population] #fitness

        logger.debug('pop: %s', str(pop))
        logger.debug('fitn: %s', str(b))

        children, parents = genetica.get_children(pop,b, len(population)+LUCKY_FEW)

        logger.debug('check5')
        logger.debug('children: %s', str(children))
        logger.debug('parents: %s', str(parents))


        lucky = genetica.select_lucky_few(list(pop), parents)


        logger.debug('check6')

        for x in lucky:
            y = convertToSim(x)

            logger.debug('lucky: %s', str(y))

            population.append(y)

        logger.debug('population after: %d', len(population))
        logger.debug('check7')

        for indiv in children:
            ind = convertToSim(indiv)
            while (1):
                mu = genetica.mutate(ind, steps_num = random.randint(0,GAIT_STEPS))
                if mu not in population:
                    population.append(mu)
                    break
        logger.debug('check8')

    print("end")
    for sim in simu:
        sim.multi_stop()


if __name__ == '__main__':
    main()
