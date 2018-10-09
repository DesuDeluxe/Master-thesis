from simulation import Simulation
from genetic import Genetic
import random
from time import sleep
from datetime import datetime
import os
import sys
import logging
#import matplotlib
#import matplotlib.pyplot as plt
#logging.basicConfig(level=logging.DEBUG)
#logging.basicConfig(level=logging.INFO)

logger = logging.getLogger(__name__)


from multiprocessing import Process, Pipe


GENERATIONS = 500
POPILATION_SIZE = 150#20
GAIT_STEPS = 4
ANGLES = 6#18 different for all joints
CHROMOSOME_SIZE = ANGLES*GAIT_STEPS
BEST_SAMPLES = 10#int(POPILATION_SIZE*0.2)
LUCKY_FEW = 5#int(BEST_SAMPLES*0.1)
MUTATION_CHANCE = 0.9
TOURNAMENT = 5
if LUCKY_FEW <0:
    LUCKY_FEW = 0

MAX_PARALLEL = 5
POP_STEPS = POPILATION_SIZE/MAX_PARALLEL


STEP_REPEAT = 4#TODO

MAX_1=50
MIN_1=-50
MAX_2=150#180
MIN_2=30 #0
MAX_3=150#160
MIN_3=30#-90
MAX_MIN = [ [MIN_1,MAX_1], [MIN_2,MAX_2], [MIN_3,MAX_3] ]


LEG_GROUP1 = [1,4,5]#TODO
LEG_GROUP2 = [2,3,6]#TODO




FITNESS_MAX_HISTORY = [0]

class Multi(object):
    def __init__(self):
        self.simok = False
        self.data = []
        self.indi = []

    def multi_start(self):
        self.parent_pipe, child_pipe = Pipe()
        self.proc = Process(target=Simulation, args=(child_pipe, GAIT_STEPS*STEP_REPEAT,))
        self.proc.start()

    def multi_stop(self):
            self.proc.join()

def newFile():
    path, filename = os.path.split(os.path.abspath(__file__))
    filename1 = path +r'generations-' + datetime.now().strftime("%m_%d_%Y_%H_%M") + ".txt"
    file = open(filename1, 'w')
    file.write("POPILATION_SIZE: %s, GAIT_STEPS: %s, MUTATION_CHANCE: %s, BEST_SAMPLES: %s, STEP_REPEAT: %s, TOURNAMENT: %s\n" % (POPILATION_SIZE, GAIT_STEPS,MUTATION_CHANCE, BEST_SAMPLES, STEP_REPEAT, TOURNAMENT))
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


def convertToSim(chrom):
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

def slicePop(pop):
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


def main():
    filename, history =newFile()
    simu = []
    for x in range(MAX_PARALLEL):
        simu.append(Multi())
        simu[-1].multi_start()
    joints_seg = simu[0].parent_pipe.recv()
    #simulation.setup()
#################### genetic
#fitness odleglosc od zrodka (x albo y) plus 10x za kazda noge na ziemi minus[kat podst - aktualny] - x3(xyx)

#10*sim_data[1][1] - (0 - abs(sim_data[2][1])) - (0 - abs(sim_data[2][2])) + [10*x for x in sim_data[0]]
    genetica = Genetic(POPILATION_SIZE, CHROMOSOME_SIZE, GAIT_STEPS, MAX_MIN, joints_seg, BEST_SAMPLES, LUCKY_FEW, TOURNAMENT, MUTATION_CHANCE)
#resetBasePositionAndOrientation
    #population = []
    population = genetica.gen_population()
    logger.debug('new pop: %s', str(population))

    #print(population)
    #population[0] = convertToSim(ppp[0])
    #population[1] = convertToSim(ppp[1])

    pop_data = []
    pop_fitness = [0,0]



    for generation in range(1,GENERATIONS):
        #FITNESS_MAX_HISTORY.append(pop_fitness[0])
        #show_plot(fig, li, generation)
        print("Generation:", str(generation) + " previous 2 best fitness: ",str(pop_fitness[0]), str(pop_fitness[1]) )
        weighted_population = []

        slicedd = slicePop(population)
        #print(len(slicedd))

        logger.debug('check1 pop: %d', len(population))

        for slic in slicedd:
            #print('.',end=" ",flush=True)
            #sys.stdout.flush()
            #sys.stdout.write('.')
            #sys.stdout.flush()

            for idx, individual in enumerate(slic):
                #print('lel')
                #print(individual)
                #print('pop')
                #print(len(population))
                #print('simu')
                #print(len(simu))
                #print(idx)
                simu[idx].indi = individual
                #print(individual)
                y = copytoLegs(individual)
                logger.debug('sent: %s', str(y))
                y = stepRepeat(y)
                logger.debug('repeated %s', y)
                logger.debug('%d', len(y))
                simu[idx].parent_pipe.send(y)

            #print('lel2')
            t=0
            while True:
                logger.debug('in loop1')
                #sys.stdout.flush()

                try:
                    for sim in simu:
                        #print('in loop1 lel')
                        if sim.simok == False:
                            rec = sim.parent_pipe.recv()
                            if rec == "simok":
                                sim.simok = True
                                t+=1
                    if t == MAX_PARALLEL:
                        t=0
                        break
                except:
                    continue
            #print('lel3')
            while True:

                logger.debug('in loop2')
                #sys.stdout.flush()

                try:
                    for sim in simu:
                        rec = sim.parent_pipe.recv()
                        contact, base_pos, base_angle = rec[-1]
                        if len(contact) == 6 and len(base_pos) == 3 and len(base_angle) == 3:
                            sim.data = rec
                            sim.parent_pipe.send('dataok')

                            logger.debug('M dataok sent')
                            #sys.stdout.flush()

                            t+=1
                    if t == MAX_PARALLEL:
                        t=0
                        break
                except:
                    continue

            for sim in simu:
                sim.simok = False
                sim.parent_pipe.send('reset')


                logger.debug('M reset sent')
                #sys.stdout.flush()

            #print('lel4')
            while True:
                logger.debug('in loop3')
                try:
                    for sim in simu:
                        rec = sim.parent_pipe.recv()
                        if rec == "resok":
                            t+=1
                    if t == MAX_PARALLEL:
                        t=0
                        break
                except:
                    continue

            for sim in simu:
                #fitness_val = genetica.fitness(sim.data[0],sim.data[1],sim.data[2])
                dd = 0
                for x in sim.data:
                    #print(dd)
                    #print(len(sim.data))
                    if x == sim.data[-1]:
                        dd+=genetica.fitness(x[0],x[1],x[2])
                    else:
                        dd+=genetica.stability(x[0], x[1], x[2])
                fitness_val = dd
                #print("lelelee")
                #print(sim.indi)
                individual = convertToGen(sim.indi)
                #print("lvggjvjhg")
                #print(len(individual))
                pair = (individual, fitness_val)
                weighted_population.append(pair)

        population = []
        logger.debug('check2')
        #for x in range(GAIT_STEPS)
        #pop_sorted, pop_fitness = genetica.sort_pop(weighted_population)
        pop_sorted,  pop_fitness = genetica.sort_pop(weighted_population)
        #pop_sort = [pop_sorted, pop_fitness]
        #print(pop_sorted[0])
        logger.debug('%s',str(pop_fitness))
        #saveToFile(str(generation)+" best fitness: " + str(pop_fitness[0]), [pop_sorted + pop_fitness])
        saveToFile(filename, "Generation: " + str(generation)+ " best fitness: " + str(pop_fitness[0]), [pop_fitness[:15], pop_sorted[:15]])
        saveToFile(history, str(generation)+ "," + str(pop_fitness[0]), [])
        #print("saved to file")
        logger.debug('check3')

        #2 parents survive
        #print(pop_sorted[1])


        a = convertToSim(pop_sorted[0])
        b = convertToSim(pop_sorted[1])
        #population = [a, b]
        #NOTE
        #print("D-1")
        #print(a)
        #print("D0")
        #print(b)
        #print("D1")
        #print(population)
        #nn = [genetica.mutate(a,1,1), genetica.mutate(a,1,1), genetica.mutate(b,1,1)]
        '''
        y=0
        while y<2:
            z = genetica.mutate(a,1,1)
            if z not in population:
                population.append(z)
                y+=1
        #print("D2")
        #print(z)
        y=0
        while y<2:
            z = genetica.mutate(b,1,1)
            if z not in population:
                population.append(z)
                y+=1
        '''
        #print("D3")
        #print(z)
        population = [a, b, genetica.mutate(a,1,1), genetica.mutate(a,1,1), genetica.mutate(b,1,1)]
        logger.debug('check4')
        #old_sel = genetica.select_population(pop_sorted)

        old_sel = pop_sorted

        #print("old_sel")
        #print(old_sel[0])
        #for ind in old_sel:
        #    population.append(convertToSim(ind))
        #old_sel = genetica.tournamen(old_s el)
        #print(len(old_sel[0]))
        #print(old_sel)
        #winners = tournamen(weighted_population)
        pop = [x[0] for x in weighted_population] #population
        b = [x[1] for x in weighted_population] #fitness
        children, parents = genetica.get_children(pop,b, len(population)+LUCKY_FEW)
        logger.debug('check5')

        lucky = genetica.select_lucky_few(list(pop), parents)
        for x in lucky:
            x = convertToSim(x)
            population.append(x)

        logger.debug('population after: %d', len(population))
        logger.debug('check6')

        #for x in range(TOURNAMENT):
        #    population.append(genetica.gen_chromosome())
        for indiv in children:
            #print("D4x")
            #print(indiv)
            ind = convertToSim(indiv)
            while True:
                #print('child loop2')
                #print("D4445")
                #print(indiv)
                #print('size')
                #print(len(indiv))
                #print("D5")
                #print(indiv)
                mu = genetica.mutate(ind)
                if mu not in population:
                    population.append(mu)
                    break
        logger.debug('check6')
        #print("lele2")
        #print("population size")
        #print(len(population))
        #print(population)
    print("end")
    for sim in simu:
        sim.multi_stop()


if __name__ == '__main__':
    main()
