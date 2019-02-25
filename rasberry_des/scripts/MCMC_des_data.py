# -*- coding: utf-8 -*-
"""
@author: abhishesh
"""
import numpy as np
#import event_log_processor.py as log1
from scipy.stats import norm
#import scipy as sp
import matplotlib.pyplot as plt
import math
#import seaborn as snss

#state_2_times = []
#data = log1.get_multi_iter_state_time_gauss(state_2_times, 2, "Picking", plot_data=True)
#mu = np.mean(data)
#sig = np.std(data)


data2= np.array([2166.8999999999996, 1090.4, 950.4000000000001, 2025.7999999999993, 2167.5, 1361.4000000000015, 672.7000000000007, 2017.5999999999985, 
         2182.1000000000004, 1624.2999999999993, 401.3999999999978, 2171.4000000000015, 2007.5, 1892.0, 268.40000000000146, 2166.5999999999985,
         2031.6000000000022, 2032.2999999999993, 132.5, 2030.3999999999978, 2017.0, 2152.899999999994, 132.8000000000029, 1887.0999999999985, 
         2152.5, 2177.5999999999985, 269.6999999999971, 1752.0, 2015.300000000003, 2011.199999999997, 667.7000000000044, 1489.300000000003, 2007.800000000003, 
         2124.5999999999985, 807.3000000000029, 1228.9000000000015, 2002.300000000003, 1895.800000000003, 1338.300000000003, 681.3000000000029, 2158.199999999997, 
         2147.800000000003, 1469.0, 545.7000000000116, 2026.199999999997, 2155.899999999994, 1749.5, 270.20000000001164, 2139.9000000000087, 2156.0, 1901.5, 
         274.6999999999971, 2030.800000000003, 2151.899999999994, 2033.199999999997, 136.59999999999127, 2004.6000000000058, 2022.800000000003, 
         2146.5, 134.1999999999971, 1896.5, 2168.800000000003, 2134.600000000006, 273.6000000000058, 1758.0, 2154.0, 2028.4000000000087, 
         534.1999999999971, 1478.1000000000058, 2168.699999999997, 2018.7000000000116, 816.3999999999942, 1337.5999999999913, 2016.7999999999884, 
         2022.3999999999942, 1067.2000000000116, 1078.1000000000058, 2159.600000000006, 2146.0, 1075.7999999999884, 1078.2999999999884, 
         2165.8000000000175, 2023.3000000000175, 1194.8999999999942, 955.6999999999825, 2021.6000000000058, 2163.100000000006, 1360.6000000000058, 
         670.6000000000058, 2157.3000000000175, 2167.2999999999884, 1486.2999999999884, 535.6000000000058, 2152.0, 2039.3000000000175, 1738.1999999999825, 
         409.29999999998836, 2168.0, 2047.0, 1896.5, 269.1000000000058, 2155.2999999999884, 2025.5999999999767, 2019.5, 133.60000000000582, 2139.600000000006, 
         2028.3999999999942, 2154.5, 2026.8999999999942, 2022.0, 2154.5, 274.0999999999767, 1746.2000000000116, 1468.2000000000116])
         
min_time = 100.*math.floor(min(data2)/100.)
max_time = 100.*math.ceil(max(data2)/100.)
print max_time-min_time, max_time, min_time 
n, bins, patches = plt.hist(data2, bins=int(math.ceil((max_time-min_time)/(0.1*(max_time-min_time)))), range=(min_time, max_time))
print bins
plt.xlabel('Time')
plt.ylabel('No. of pickers')
plt.title('All pickers')
plt.show()
mu = np.mean(data2)
sigma = np.std(data2)
print mu, sigma
x=np.arange(min_time, max_time, .1)
plt.plot(x, 1/(sigma * np.sqrt(2 * np.pi)) * np.exp( - (x - mu)**2 / (2 * sigma**2) ), linewidth=2, color='r')
plt.xlabel('Time(in msec)')
plt.ylabel('Probability')
plt.title('Prio Probability distribution - STATE 2')
plt.show()



data1= np.array([1.7, 80.40000000000009, 126.19999999999982, 35.900000000000546, 107.79999999999927, 52.5, 4.700000000000728,
 26.100000000000364, 97.89999999999964, 59.60000000000218, 4.600000000002183, 16.399999999997817, 96.20000000000073, 
 70.19999999999709, 4.599999999998545, 11.599999999998545, 90.59999999999854, 72.39999999999782, 4.700000000000728, 
 6.5, 81.29999999999563, 82.90000000000146, 6.599999999998545, 4.5, 72.30000000000291, 88.70000000000437, 11.30000000000291, 
 4.5, 63.19999999999709, 101.80000000000291, 26.5, 4.5, 56.599999999998545, 111.60000000000582, 30.80000000000291,
 4.799999999995634, 44.599999999998545, 117.30000000000291, 49.80000000000291, 4.600000000005821, 25.60000000000582, 
 104.69999999999709, 55.69999999999709, 4.599999999991269, 21.09999999999127, 97.70000000001164, 64.60000000000582, 4.5, 
 11.099999999991269, 91.5, 70.70000000001164, 4.5, 11.599999999991269, 83.30000000000291, 76.69999999999709, 4.5,
 6.19999999999709, 79.90000000000873, 84.30000000000291, 6.7000000000116415, 4.5, 75.30000000000291, 90.39999999999418, 
 11.39999999999418, 4.7000000000116415, 66.40000000000873, 95.59999999999127, 20.89999999999418, 4.599999999991269, 
 56.80000000000291, 105.29999999998836, 29.80000000000291, 4.5, 51.20000000001164, 112.5, 40.5, 4.7000000000116415, 
 40.79999999998836, 118.79999999998836, 40.0, 4.7000000000116415, 41.39999999999418, 114.19999999998254, 45.39999999999418,
 4.600000000005821, 37.0, 108.70000000001164, 49.39999999999418, 4.599999999976717, 27.0, 103.20000000001164, 
 54.89999999999418, 4.5, 21.199999999982538, 99.59999999997672, 67.0, 4.5, 16.5, 96.30000000001746, 70.59999999997672, 
 4.5, 11.5, 84.10000000000582, 73.89999999999418, 4.599999999976717, 6.699999999982538, 89.39999999999418, 76.70000000001164, 
 3.1000000000058208, 72.70000000001164, 91.09999999997672, 11.700000000011642, 4.5, 65.59999999997672])

min_time = 100.*math.floor(min(data1)/100.)
max_time = 100.*math.ceil(max(data1)/100.)
print max_time-min_time, max_time, min_time 
n, bins, patches = plt.hist(data1, bins=int(math.ceil((max_time-min_time)/(0.1*(max_time-min_time)))), range=(min_time, max_time))
print bins
plt.xlabel('Time')
plt.ylabel('No. of pickers')
plt.title('All pickers')
plt.show()
mu = np.mean(data1)
sigma = np.std(data1)
print mu, sigma
x=np.arange(min_time, max_time, .1)
plt.plot(x, 1/(sigma * np.sqrt(2 * np.pi)) * np.exp( - (x - mu)**2 / (2 * sigma**2) ), linewidth=2, color='r')
plt.xlabel('Time(in msec)')
plt.ylabel('Probability')
plt.title('Prio Probability distribution - STATE 1')
plt.show()


     # Calculate the Posterior of the sample data so that it can be taken as reference for sampling from Posterior using likelihood * prior 
def cal_post_analytical(data, x, mu_0, sigma_0):            # x is equivalent to each STATE data samples 
    sigma = 1.
    n = len(data)
    mu_posterior = (mu_0 / sigma_0**2 + data.sum() / sigma**2) / (1. / sigma_0**2 + n / sigma**2)
    sigma_posterior = (1. / sigma_0**2 + n / sigma**2)**-1
    return norm(mu_posterior, np.sqrt(sigma_posterior)).pdf(x)

def MCMC_Metro(data, prior_mu=np.mean(data1), prior_std=np.std(data1), mean_target=np.mean(data2), std_target=np.mean(data2),iterations=50, plot=False):

 
    mu_current_evidence = mean_target
    posterior = [mu_current_evidence]
    for i in range(iterations):
        # suggest new position
        mu_transition_proposal = norm(mu_current_evidence, std_target).rvs()    # random variates of given size 

        # Compute likelihood by multiplying probabilities of each data point
        current_likelihood = norm(mu_current_evidence, 1.).pdf(data).prod()
        proposal_likelihood = norm(mu_transition_proposal, 1.).pdf(data).prod()
        
        # Compute prior probability of current and proposed mu        
        current_prior = norm(prior_mu, prior_std).pdf(mu_current_evidence)
        proposal_prior = norm(prior_mu, prior_std).pdf(mu_transition_proposal)
        
        current_prob = (current_likelihood * current_prior)
        proposal_prob = (proposal_likelihood * proposal_prior)
        print current_prob, proposal_prob
        # Accept proposal?
        acceptance_prob = proposal_prob / current_prob
        
        # Usually would include prior probability, which we neglect here for simplicity
        accept = np.random.rand() < acceptance_prob
        
        if accept:
            # Update position
            mu_current_evidence = mu_transition_proposal
        
        posterior.append(mu_current_evidence)
        
    return posterior

if __name__ == '__main__':
#    data2 = np.random.uniform(low=0., high=25500., size=2000)
#    print data2
    trace = [50]
    MCMC_Metro(data2, plot=True)
    cal_post_analytical(data2, 200, mu, sigma)



