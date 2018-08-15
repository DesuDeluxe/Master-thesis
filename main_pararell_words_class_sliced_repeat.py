from simulation import Simulation
from genetic import Genetic
import random
from time import sleep
from datetime import datetime
import os
import sys

from multiprocessing import Process, Pipe


GENERATIONS = 5000
POPILATION_SIZE = 100#20
GAIT_STEPS = 6
ANGLES = 18
CHROMOSOME_SIZE = ANGLES*GAIT_STEPS
BEST_SAMPLES = 5#int(POPILATION_SIZE*0.2)
LUCKY_FEW = 2#int(BEST_SAMPLES*0.1)
MUTATION_CHANCE = 0.7
TOURNAMENT = BEST_SAMPLES
if LUCKY_FEW <0:
    LUCKY_FEW = 0

MAX_PARALLEL = 10
POP_STEPS = POPILATION_SIZE/MAX_PARALLEL


MAX_1=50
MIN_1=-50
MAX_2=150#180
MIN_2=30 #0
MAX_3=150#160
MIN_3=30#-90
MAX_MIN = [ [MIN_1,MAX_1], [MIN_2,MAX_2], [MIN_3,MAX_3] ]

ppp = [

[-40, 140, 70, 10, 80, 80, -20, 80, 30, -40, 50, 40, -10, 80, 110, -30, 60, 40, 40, 110, 50, -30, 130, 30, 40, 90, 100, 40, 70, 80, -30, 80, 50, 40, 90, 80, 40, 50, 90, 30, 60, 90, 20, 60, 70, 40, 140, 100, 20, 30, 90, -10, 80, 30, 40, 110, 40, -40, 120, 60, -30, 130, 110, -40, 140, 140, -10, 40, 90, -20, 140, 90, 0, 50, 100, -50, 30, 30, 20, 70, 30, -30, 50, 100, 0, 110, 70, 20, 60, 90, -20, 30, 130, 20, 70, 70, 20, 50, 100, 10, 70, 60, 30, 140, 140, 0, 30, 130],
[-40, 140, 70, 10, 80, 80, -20, 80, 30, -40, 50, 40, -10, 80, 110, -30, 60, 40, 40, 110, 50, -30, 130, 30, 40, 90, 100, 40, 70, 80, -30, 80, 50, 40, 90, 80, 40, 50, 90, 30, 60, 90, 20, 60, 70, 40, 140, 100, 20, 30, 90, -10, 80, 30, 40, 110, 40, -40, 120, 60, -30, 130, 110, -40, 140, 140, -10, 40, 90, -20, 140, 90, 0, 50, 100, -50, 30, 30, 20, 70, 30, -30, 50, 100, 0, 110, 70, 20, 60, 90, -20, 30, 130, 20, 70, 70, 20, 50, 100, 10, 70, 60, 30, 140, 140, 0, 30, 130]


]


class Multi(object):
    def __init__(self):
        self.simok = False
        self.data = []
        self.indi = []

    def multi_start(self):
        self.parent_pipe, child_pipe = Pipe()
        self.proc = Process(target=Simulation, args=(child_pipe, GAIT_STEPS,))
        self.proc.start()

    def multi_stop(self):
            self.proc.join()

def newFile():
    path, filename = os.path.split(os.path.abspath(__file__))
    filename = path +"\\generations-" + datetime.now().strftime("%m_%d_%Y_%H_%M") + ".txt"
    file = open(filename, 'w')
    file.write("POPILATION_SIZE: %s, GAIT_STEPS: %s, MUTATION_CHANCE: %s, BEST_SAMPLES: %s\n" % (POPILATION_SIZE, GAIT_STEPS,MUTATION_CHANCE, BEST_SAMPLES))
    file.close()
    return filename

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
        a =chrom[x:x+ANGLES]
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

def main():
    filename =newFile()
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
    population = []
    population = genetica.gen_population()
    #print(population)
    #population[0] = convertToSim(ppp[0])
    #population[1] = convertToSim(ppp[1])

    pop_data = []
    pop_fitness = [0,0]
    for generation in range(GENERATIONS):
        print("Generation:", str(generation) + " previous 2 best fitness: ",str(pop_fitness[0]), str(pop_fitness[1]) )
        weighted_population = []

        slicedd = slicePop(population)
        #print(len(slicedd))
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
                simu[idx].parent_pipe.send(individual)

            #print('lel2')
            t=0
            while True:
                #print('in loop1')
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
                #print('in loop2')
                try:
                    for sim in simu:
                        rec = sim.parent_pipe.recv()
                        contact, base_pos, base_angle = rec[-1]
                        if len(contact) == 6 and len(base_pos) == 3 and len(base_angle) == 3:
                            sim.data = rec
                            sim.parent_pipe.send('dataok')
                            t+=1
                    if t == MAX_PARALLEL:
                        t=0
                        break
                except:
                    continue

            for sim in simu:
                sim.simok = False
                sim.parent_pipe.send('reset')
            #print('lel4')
            while True:
                #print('in loop3')
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
        #for x in range(GAIT_STEPS)
        pop_sorted, pop_fitness = genetica.sort_pop(weighted_population)
        #print(pop_sorted[0])
        print(pop_fitness)
        #saveToFile(str(generation)+" best fitness: " + str(pop_fitness[0]), [pop_sorted + pop_fitness])
        saveToFile(filename, "Generation: " + str(generation)+ " best fitness: " + str(pop_fitness[0]), [pop_fitness[:15], pop_sorted[:15]])
        #print("saved to file")


        #2 parents survive
        a = convertToSim(pop_sorted[0])
        b = convertToSim(pop_sorted[1])
        population = [a, b]
        #print("D-1")
        #print(a)
        #print("D0")
        #print(b)
        #print("D1")
        #print(population)
        #nn = [genetica.mutate(a,1,1), genetica.mutate(a,1,1), genetica.mutate(b,1,1)]
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
        #print("D3")
        #print(z)
        #population = [a, b, genetica.mutate(a,1,1), genetica.mutate(a,1,1), genetica.mutate(b,1,1)]

        old_sel = genetica.select_population(pop_sorted)
        #print("old_sel")
        #print(old_sel[0])
        #for ind in old_sel:
        #    population.append(convertToSim(ind))
        #old_sel = genetica.tournamen(old_s el)
        #print(len(old_sel[0]))
        #print(old_sel)


        children = genetica.get_children(old_sel, len(population))

        #for x in range(TOURNAMENT):
        #    population.append(genetica.gen_chromosome())

        for indiv in children:
            #print("D4x")
            #print(indiv)
            while True:
                #print('child loop2')
                #print("D4445")
                #print(indiv)
                #print('size')
                #print(len(indiv))
                ind = convertToSim(indiv)
                #print("D5")
                #print(indiv)
                mu = genetica.mutate(ind)
                if mu not in population:
                    population.append(mu)
                    break

        #print("lele2")
        #print("population size")
        #print(len(population))
        #print(population)
    print("end")
    for sim in simu:
        sim.multi_stop()


if __name__ == '__main__':
    main()
