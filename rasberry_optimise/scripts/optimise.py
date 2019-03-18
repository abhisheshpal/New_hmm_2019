#!/usr/bin/env python
from __future__ import division
import rospy, sys, random, time, datetime, pickle, os, numpy as np
from rasberry_optimise.utils import *
from rasberry_optimise.rasberry_scenario_server import scenario_server
from deap import base, creator, tools, algorithms
import rospkg


def evaluate(individual):
    """Run the test scenario (move from start node to goal node) given a set of
       parameters (individual) and get fitness metrics.
    """

    # Make dictionary of parameters to pass to the scenario server.
    params = make_param_dict(config_params, individual)

    metric_array = np.empty((NUM_RUNS, 6))
    for i in range(NUM_RUNS):

        time_1 = time.time()
        metrics, trajectory, trajectory_ground_truth, trajectory_amcl \
        = ss.run_scenario(params)
        time_2 = time.time()
        metric_array[i, :] = metrics

        eval_calls.append(1)
        times.append(time_2-time_1)
        data.append([individual, metrics])

        evals_remaining = tot_eval_calls - np.sum(eval_calls)
        time_to_complete = (evals_remaining * np.mean(times)) / 3600.0

        print "Evaluations remaining (estimated): {}/{}".format(evals_remaining, tot_eval_calls)
        print "Estimated time to complete: {} hours".format(time_to_complete)


    t = np.median(metric_array[:, 0])
    rotation_cost = np.median(metric_array[:, 1])
    trajectory_length = np.median(metric_array[:, 2])
    dist_from_coords = np.median(metric_array[:, 3])
    position_error = np.median(metric_array[:, 4])
    orientation_error = np.median(metric_array[:, 5])

    metrics = (t, rotation_cost, trajectory_length, dist_from_coords, position_error, orientation_error)

    if MULTI_OBJECTIVE:
        metrics = (np.sum(np.array(weights_multi) * np.array(metrics)),)

    return metrics


def checkBounds(mins, maxs):
    """Make sure that crossover/mutation doesn't produce a parameter that falls out
       of bounds.
    """
    def decorator(func):
        def wrapper(*args, **kargs):
            offspring = func(*args, **kargs)
            for child in offspring:
                for i in xrange(len(child)):
                    if child[i] > maxs[i]:
                        child[i] = maxs[i]
                    elif child[i] < mins[i]:
                        child[i] = mins[i]
            return offspring
        return wrapper
    return decorator
#####################################################################################


#####################################################################################
if __name__ == "__main__":

    rospy.init_node("optimiser", anonymous=True, disable_signals=True)

    if len(sys.argv) < 4:
        rospy.loginfo("usage is optimise.py path_to_scenario_yaml path_to_parameters_yaml path_to_ga_yaml")
        exit()
    else:
        print sys.argv
        print "\n"
        config_scenario_path = sys.argv[1]
        config_parameters_path = sys.argv[2]
        config_ga_path = sys.argv[3]

    # Get configuration for the optimisation procedure.
    config_scenario = load_data_from_yaml(config_scenario_path)
    config_params = load_data_from_yaml(config_parameters_path)
    config_ga = load_data_from_yaml(config_ga_path)
#####################################################################################


#####################################################################################
    # Set hyper-parameters of the GA.
    NGEN = config_ga["ngen"]
    POPSIZE = config_ga["init_popsize"]
    INDPB = config_ga["indpb"]
    C = config_ga["c"]
    TOURNSIZE = config_ga["tournsize"]
    MU = config_ga["mu"]
    LAMBDA_ = config_ga["lambda"]
    CXPB = config_ga["cxpb"]
    MUTPB = config_ga["mutpb"]
    WEIGHT_TIME = config_ga["weight_time"]
    WEIGHT_ROTATION = config_ga["weight_rotation"]
    WEIGHT_LENGTH = config_ga["weight_length"]
    WEIGHT_PATH_ERROR = config_ga["weight_path_error"]
    WEIGHT_POSITION_ERROR = config_ga["weight_position_error"]
    WEIGHT_ORIENTATION_ERROR = config_ga["weight_orientation_error"]
    MULTI_OBJECTIVE = config_ga["multi_objective"]
    NUM_RUNS = config_ga["num_runs"]

    print "\nSetting hyper-parameters of the genetic algorithm ..."
    print "Setting ngen = {}".format(NGEN)
    print "Setting popsize = {}".format(POPSIZE)
    print "Setting indpb = {}".format(INDPB)
    print "Setting c = {}".format(C)
    print "Setting tournsize = {}".format(TOURNSIZE)
    print "Setting mu = {}".format(MU)
    print "Setting lambda = {}".format(LAMBDA_)
    print "Setting cxpb = {}".format(CXPB)
    print "Setting mutpb = {}".format(MUTPB)
    print "Setting weight_time = {}".format(WEIGHT_TIME)
    print "Setting weight_rotation = {}".format(WEIGHT_ROTATION)
    print "Setting weight_length = {}".format(WEIGHT_LENGTH)
    print "Setting weight_path_error = {}".format(WEIGHT_PATH_ERROR)
    print "Setting weight_position_error = {}".format(WEIGHT_POSITION_ERROR)
    print "Setting weight_orientation_error = {}".format(WEIGHT_ORIENTATION_ERROR)
    print "Setting multi_objective = {}".format(MULTI_OBJECTIVE)
    print "Setting num_runs = {}".format(NUM_RUNS)
#####################################################################################


#####################################################################################
    # Create DEAP toolbox and register parameters for optimisation.
    weights = (WEIGHT_TIME, WEIGHT_ROTATION, WEIGHT_LENGTH, WEIGHT_PATH_ERROR,
               WEIGHT_POSITION_ERROR, WEIGHT_ORIENTATION_ERROR)

    if MULTI_OBJECTIVE:
        weights_multi = weights
        weights = (1.0,)

    creator.create("FitnessMulti", base.Fitness, weights=weights)
    creator.create("Individual", list, fitness=creator.FitnessMulti)
    toolbox = base.Toolbox()

    rcnfsrvs = config_params.keys()
    attributes = []
    sigmas = [] # std of gaussian mutation
    mins = []
    maxs = []

    print "\nRegistering the following parameters for optimisation ..."
    for i, rcnfsrv in enumerate(rcnfsrvs):
        param_names = config_params.values()[i].keys()

        for param_name in param_names:
            attr = rcnfsrv + "/" + param_name
            print attr

            if config_params.values()[i][param_name]['type'] == "float":
                attr_min = config_params.values()[i][param_name]['min']
                attr_max = config_params.values()[i][param_name]['max']
                toolbox.register(attr, random.uniform, attr_min, attr_max)

                sigmas.append((0.5 * (attr_max - attr_min)) / C)
                mins.append(attr_min)
                maxs.append(attr_max)

            elif config_params.values()[i][param_name]['type'] == "int":
                attr_min = config_params.values()[i][param_name]['min']
                attr_max = config_params.values()[i][param_name]['max']
                toolbox.register(attr, random.randint, attr_min, attr_max)

                sigmas.append((0.5 * (attr_max - attr_min)) / C)
                mins.append(attr_min)
                maxs.append(attr_max)

            elif config_params.values()[i][param_name]['type'] == "bool":
                toolbox.register(attr, random.randint, 0, 1)

                sigmas.append(1.0)
                mins.append(0)
                maxs.append(1)

            else:
                print "Please set {} to type `float`, `int` or `bool` in parameter configuration file.".format(param_name)
                sys.exit()

            attributes.append(toolbox.__getattribute__(attr))

    print "\n"
#####################################################################################


#####################################################################################
    # Register the DEAP operators.

    toolbox.register("individual", tools.initCycle, creator.Individual, attributes, n=1)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    toolbox.register("mate", tools.cxTwoPoint)
    toolbox.register("mutate", tools.mutGaussian, mu=[0]*len(sigmas), sigma=sigmas,
                     indpb=INDPB)

    toolbox.decorate("mate", checkBounds(mins, maxs))
    toolbox.decorate("mutate", checkBounds(mins, maxs))

    if TOURNSIZE == 0:
        toolbox.register("select", tools.selBest)
    else:
        toolbox.register("select", tools.selTournament, tournsize=TOURNSIZE)

    toolbox.register("evaluate", evaluate)
#####################################################################################


#####################################################################################
    # For recording the best params (hall of fame) and logging statistics.
    hof = tools.HallOfFame(100)
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", np.mean, axis=0)
    stats.register("std", np.std, axis=0)
    stats.register("min", np.min, axis=0)
    stats.register("max", np.max, axis=0)
#####################################################################################


#####################################################################################
    # Initialise the scenario server and run the genetic algorithm.
    rcnfsrv1 = "/move_base/local_costmap/local_inflation_layer"
    rcnfsrv2 = "/move_base/global_costmap/global_inflation_layer"

    if rcnfsrv1 in rcnfsrvs and rcnfsrv2 not in rcnfsrvs:
        rcnfsrvs.append(rcnfsrv2)

    ss = scenario_server(config_scenario, rcnfsrvs)

    eval_calls = []
    times = []
    data = []
    tot_eval_calls = NUM_RUNS * (int(NGEN * np.round(LAMBDA_ * (CXPB + MUTPB))) + POPSIZE) # average
    initial_pop = toolbox.population(POPSIZE)

    pop, logbook = algorithms.eaMuPlusLambda(initial_pop, toolbox, mu=MU,
    lambda_=LAMBDA_, cxpb=CXPB, mutpb=MUTPB, ngen=NGEN, stats=stats, halloffame=hof,
    verbose=True)
#####################################################################################


#####################################################################################
    # Save data.
    rospack = rospkg.RosPack()
    base_dir = rospack.get_path("rasberry_optimise")
    if "save_path" in config_ga.keys():
        save_path = config_ga["save_path"]
    else:
        save_path = base_dir

    save_dir = os.path.join(save_path,
    datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
    os.mkdir(save_dir)

    save_data_to_json(save_dir + "/hof.json", [ind for ind in hof])
    save_data_to_json(save_dir + "/pop.json", pop)
    save_data_to_json(save_dir + "/data.json", data)
    pickle.dump(logbook, open(save_dir + "/logbook.p", "wb"))

    save_data_to_yaml(save_dir + "/config_scenario.yaml", config_scenario)
    save_data_to_yaml(save_dir + "/config_params.yaml", config_params)
    save_data_to_yaml(save_dir + "/config_ga.yaml", config_ga)
#####################################################################################