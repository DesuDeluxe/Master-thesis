import random
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
      return population

    def gen_chromosome(self):
        chromosome = []
        for c in range(0, self.gait_steps):
          chromosome.append(self.random_angles(self.joints_seg))
        return chromosome

    def get_children(self, pop, ll):
        nextPopulation = []
        #for x in range(self.best_samples):
        #nextPopulation.append(pop[0])
        #for x in range(int(self.best_samples/2)):
        #for x in range(self.best_samples):

        ch1, ch2 = self.crossover(pop[0], pop[1])
        nextPopulation.append(ch1)
        nextPopulation.append(ch1)
        target = self.population_size - ll#survivors
        #ch1, ch2 = self.crossover(pop[2*x-1], pop[2*x])
        while len(nextPopulation) < target:
            #print("lele4")
            #print('child loop1')
            random.seed()
            a = random.sample(pop, 2)
            #print('loop')
            if a[0]!=a[1]:
                ch1, ch2 = self.crossover(a[0], a[1])
                nextPopulation.append(ch1)
                if len(nextPopulation) == target:
                    break
                nextPopulation.append(ch2)
            else:
                continue

        return nextPopulation

    def sort_pop(self, pop):
        s=[]
        f=[]
        pop_sorted = sorted(pop, key=lambda pop: pop[1], reverse=True)
        for x in pop_sorted:
            s.append(x[0])
            f.append(x[1])
        return s, f

    def select_population(self, pop):
        nextGeneration = []
        #random.seed()
        #list = random.sample(pop_sorted,self.best_samples)
        for x in range(self.best_samples):
            nextGeneration.append(pop[x])
        if self.lucky_few > 0:
            random.seed()
            list = random.sample(pop[self.best_samples:-1],self.lucky_few)
            for i in list:
                nextGeneration.append(i)
        #random.shuffle(nextGeneration)
        return nextGeneration

    def tournamen(self, pop):
        champ = []
        for x in pop:
            competitors = random.sample(pop,self.tournament)
            competitors.sort()
            #print(competitors)
            if random.random() < 0.7:
                champ.append(competitors[0])
            else:
                champ.append(random.choice(competitors[1:]))
        return champ

    def random_angles(self,joints_seg): #6 legs, each composed of 3 links
        angles = []
        for leg in joints_seg:
            for part in range(0,len(leg)):
                random.seed()
                angles.append(random.randrange(self.max_min[part][0],self.max_min[part][1], 10))
                #print(angles[-1])
        return angles

    def random_angle(self, position):
        random.seed()
        rnd_a = self.random_angles(self.joints_seg)
        return rnd_a[position]

    def stability(self,contact, base_pos, base_angle):
        #contact
        #cont = 10*sum(contact)

        if contact[0] == 1 and contact[3] == 1 and contact[4] == 1:
            cont = 500
        elif contact[1] == 1 and contact[2] == 1 and contact[5] == 1:
            cont = 500
        else:
            cont=-500
        if base_pos[2]>0.09 and base_pos[2]<0.152:
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
            #if random.random() < self.mutation_chance: #mutate each step
                #print('mutation')
                chromosome_step.append(self.random_angle(c))
            else:
                #print('nooooottt  mutation')
                #print(step)
                #print(c)
                #print(len(chromosome))
                #print(len(chromosome[0]))
                #print(chromosome)
                chromosome_step.append(chromosome[step][c])
          chromosome_out.append(chromosome_step)
          chromosome_step = []
            #print(chromosome_step)
        return chromosome_out


    def crossover(self,dna1, dna2):
        random.seed()
        pos = random.randint(0,self.chromosome_size)
        return (dna1[:pos]+dna2[pos:], dna2[:pos]+dna1[pos:])
'''
    def crossoverr(self,dna1, dna2):
        random.seed()
        a = random.randint(0,self.gait_steps) #mutate only one step
            pos = random.randint(0,self.chromosome_size)
        return (dna1[:pos]+dna2[pos:], dna2[:pos]+dna1[pos:])
'''
