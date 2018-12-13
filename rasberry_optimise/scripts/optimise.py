#!/usr/bin/env python
import rospy, sys, random
from rasberry_optimise.utils import *
from rasberry_optimise.rasberry_scenario_server import scenario_server
from deap import base, creator, tools


def evaluate(individual):
    
    print "\n"
    params = {}
    count = 0
    for i, rcnfsrv in enumerate(rcnfsrvs):
        params[rcnfsrv] = {}
        param_names = config_parameters.values()[i].keys()
        for param_name in param_names:
            if config_parameters.values()[i][param_name]['include']:
                params[rcnfsrv][param_name] = individual[count]
                print "Setting {} = {}".format(param_name, params[rcnfsrv][param_name])
                count+=1
    
    return ss.run_scenario(params)
    
    
def deap_ga():
    pop = toolbox.population(n=50)
    CXPB, MUTPB, NGEN = 0.5, 0.2, 40

    # Evaluate the entire population
    fitnesses = map(toolbox.evaluate, pop)
    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit

    for g in range(NGEN):
        # Select the next generation individuals
        offspring = toolbox.select(pop, len(pop))
        # Clone the selected individuals
        offspring = map(toolbox.clone, offspring)

        # Apply crossover and mutation on the offspring
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < CXPB:
                toolbox.mate(child1, child2)
                del child1.fitness.values
                del child2.fitness.values

        for mutant in offspring:
            if random.random() < MUTPB:
                toolbox.mutate(mutant)
                del mutant.fitness.values

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        # The population is entirely replaced by the offspring
        pop[:] = offspring

    return pop
#####################################################################################


#####################################################################################
if __name__ == "__main__":
    

    rospy.init_node("optimiser", anonymous=True, disable_signals=True)

    if len(sys.argv) < 3:
        rospy.loginfo("usage is optimise.py path_to_scenario_yaml path_to_parameters_yaml")
        exit()
    else:
        print sys.argv
        scenario = sys.argv[1]
        parameters = sys.argv[2]

    config_scenario = load_config_from_yaml(scenario)
    config_parameters = load_config_from_yaml(parameters)
    
    # Probably load a config file for the genetic algorithm here.
    
    # Initialise the scenario server (fitness function).
    rcnfsrvs = config_parameters.keys()
    ss = scenario_server(config_scenario, rcnfsrvs)

    # Initialise the genetic algorithm
    IND_SIZE=0
    for i, rcnfsrv in enumerate(rcnfsrvs):
        param_names = config_parameters.values()[i].keys()
        for param_name in param_names:
            if config_parameters.values()[i][param_name]['include']:
                IND_SIZE+=1    
                
    creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
    creator.create("Individual", list, fitness=creator.FitnessMin)

    toolbox = base.Toolbox()
    FLOAT_MIN, FLOAT_MAX = 5.0, 50.0
    toolbox.register("attribute", random.uniform, FLOAT_MIN, FLOAT_MAX)
    toolbox.register("individual", tools.initRepeat, creator.Individual,
                     toolbox.attribute, n=IND_SIZE)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    toolbox.register("mate", tools.cxTwoPoint)
    toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1, indpb=0.1)
    toolbox.register("select", tools.selTournament, tournsize=2)
    toolbox.register("evaluate", evaluate)

    # Run the genetic algorithm
    pop = deap_ga()
#####################################################################################