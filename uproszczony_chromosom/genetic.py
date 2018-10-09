import random
import logging


#logging.basicConfig(level=logging.DEBUG)
#logging.basicConfig(level=logging.INFO)

logger = logging.getLogger(__name__)

ANGLE_STEP = 15#TODO
class Genetic(object):

    def __init__(self, population_size, chromosome_size ,gait_steps, max_min, joints_seg, best_samples, lucky_few, tournament, mutation_chance):
        self.population_size = population_size
        self.chromosome_size = chromosome_size
        self.gait_steps = gait_steps
        self.max_min = max_min
        self.joints_seg = joints_seg
        self.best_samples = best_samples
        self.lucky_few = lucky_few
        self.tournament = tournament
        self.mutation_chance = mutation_chance

    def gen_population(self):
      population = []
      for i in range(0,self.population_size):
        population.append(self.gen_chromosome())
      logger.debug('pop %s', str(population))
      return population

    def gen_chromosome(self):
        chromosome = []
        for c in range(0, self.gait_steps):#number of steps for each leg
            y=[]
            for x in range(2):#number of angles to generate - 2 legs
                y+=self.random_angles(self.joints_seg)
            chromosome.append(y)
        logger.debug('chromosome %s', str(chromosome))
        return chromosome

    def get_children(self, pop,pop_fitn, ll):
        nextPopulation = []
        parents = []
        target = self.population_size - ll#survivors
        while len(nextPopulation) < target:
            #print("lele4")
            #print('child loop1')
            winners = self.tournamen(pop, pop_fitn)
            #print('loop')
            parents+=winners
            ch1, ch2 = self.crossover(winners[0], winners[1])
            nextPopulation.append(ch1)
            if len(nextPopulation) == target:
                break
            nextPopulation.append(ch2)

        return nextPopulation, parents

    def sort_pop(self, pop):
        s=[]
        f=[]
        pop_sorted = sorted(pop, key=lambda pop: pop[1], reverse=True)
        #return pop_sorted
        for x in pop_sorted:
            s.append(x[0])
            f.append(x[1])
        return s, f

    def select_lucky_few(self, pop, parents):
        lucky = []
        #print(len(pop))
        pop = [x for x in pop if x not in parents]
        #print(len(pop))
        for x in range(self.lucky_few):
            while True:
                luck = random.choice(pop)
                if luck not in lucky:
                    lucky.append(luck)
                    break
                #print('stuck')
        return lucky

    def tournamen(self, pop, pop_fitn):
        winners = []
        for x in range(2):
            #lis = [x[1] for x in pop] #list comprehension
            competitors = random.sample(pop_fitn,self.tournament)
            #print(competitors)
            #competitors.sort()
            #print(competitors)
            winner = max(competitors)
            win_index = pop_fitn.index(winner)
            winners.append(pop[win_index])
        return winners
    '''
    def random_angles(self,joints_seg): #6 legs, each composed of 3 links
        angles = []
        #print(joints_seg)
        for leg in joints_seg:
            for part in range(0,len(leg)):
                random.seed()
                angles.append(random.randrange(self.max_min[part][0],self.max_min[part][1], ANGLE_STEP))
                #print(angles[-1])
        return angles
    '''
    def random_angles(self,joints_seg): #gen 2 sets of angles and copy to te rest of legs
        angles = []
        for part in range(0, len(joints_seg[0])):
            angles.append(random.randrange(self.max_min[part][0],self.max_min[part][1], ANGLE_STEP))

        logger.debug('angles: %s', str(angles))
        return angles


    def random_angle(self, position):
        random.seed()
        rnd_a = self.random_angles(self.joints_seg)
        rnd_b = self.random_angles(self.joints_seg)
        rnd = rnd_a+rnd_b
        logger.debug('random angles: %s', str(rnd_a))
        logger.debug('random position: %d', position)
        return rnd[position]

    def stability(self,contact, base_pos, base_angle):
        #contact
        #cont = 10*sum(contact)

        if contact[0] == 1 and contact[3] == 1 and contact[4] == 1:
            cont = 500
        elif contact[1] == 1 and contact[2] == 1 and contact[5] == 1:
            cont = 500
        else:
            cont=-500
        if base_pos[2]>0.1 and base_pos[2]<0.152:
            a = 10*base_pos[2]
        else:
            a = -10*base_pos[2]
        #angle
        angle_error = 0
        if abs(base_angle[0]) >50 or abs(base_angle[1]) >50:
            angle_error = 500
        else:
            for idx, c in enumerate(base_angle):
                if idx == 2:
                    angle_error+=abs(0 - c)
                else:
                    angle_error+=10*abs(0 - c)
        return cont + a - abs(angle_error)


    def fitness(self, contact, base_pos, base_angle):
        #[contact, base_pos, base_angle] -> [[0, 0, 0, 0, 0, 0], [-0.0, 0.0, 0.2], [0, 0, 0]]
        stab = self.stability(contact, base_pos, base_angle)
        #distance
        distance = 100*base_pos[0]
        off_path = 100*abs(base_pos[1])

        cont = 10*sum(contact)
        if cont == 6:
            cont = 1500

        return  distance - off_path + stab + cont

#must be in simulatiion format(list of steps)
    def mutate(self, chromosome, chance_override = None , all_steps = None):
        if chance_override ==1:
            mut = chance_override
        else:
            mut = self.mutation_chance

        if all_steps ==1:
            a = []
            for x in range(self.gait_steps):
                a.append(x)
        else:
            a = [random.randint(0,self.gait_steps), -5] #mutate only one step
        chromosome_step = []
        chromosome_out = []

        for step in range(0, self.gait_steps):
          for c in range(0, int(self.chromosome_size/self.gait_steps)):
            random.seed()
            if (step in a) and (random.random() < mut): #mutate only one step
                chromosome_step.append(self.random_angle(c))
            else:
                chromosome_step.append(chromosome[step][c])
          chromosome_out.append(chromosome_step)
          chromosome_step = []
            #print(chromosome_step)
        return chromosome_out


    def crossover(self,dna1, dna2):
        random.seed()
        pos = random.randint(0,self.chromosome_size)
        return (dna1[:pos]+dna2[pos:], dna2[:pos]+dna1[pos:])
