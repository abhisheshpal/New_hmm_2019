# -*- coding: utf-8 -*-
"""
Created on Sun Feb 24 17:00:35 2019

@author: abhishesh
"""
import numpy as np
import event_log_processor.py as log1
from scipy.stats import norm
#import scipy as sp
#import matplotlib.pyplot as plt
#import math
#import seaborn as snss

state_2_times = []
data = log1.get_multi_iter_state_time_gauss(state_2_times, 2, "Picking", plot_data=True)
mu = np.mean(data)
sig = np.std(data)


#calculate the posterior of the sampled data x
def cal_post_analytical(data, x, mu_0, sigma_0):            # x is equivalent to each STATE data samples 
    sigma = 1.
    n = len(data)
    mu_posterior = (mu_0 / sigma_0**2 + data.sum() / sigma**2) / (1. / sigma_0**2 + n / sigma**2)
    sigma_posterior = (1. / sigma_0**2 + n / sigma**2)**-1
    return norm(mu_posterior, np.sqrt(sigma_posterior)).pdf(x)

# This is created for state2 Picking( prior belief) >----to---> state5 tray full event 

def MCMC_Metro(data, iterations=10, mean_target=0.5, std_target=0.5, plot=False, prior_mu=.5, prior_std=.5 ):   # prior_mu is mean of previous belief 
   
    mu_current_evidence = mean_target   # mean of evidence or current distriution
    posterior = [mu_current_evidence]   # based on current evidence and prior belief   # The idea behind this thinking is to keep updating the beliefs as more evidence is provided. 
    for i in range(iterations):
        
        # With each new iteration we suggest a new position of the distribution
        mu_transition_proposal = norm(mu_current_evidence, std_target).rvs()    # random variates of given size 

        # Compute transition likelihood of with product of probabilities of each data point
        current_likelihood = norm(mu_current_evidence, 1.).pdf(data).prod()
        proposal_likelihood = norm(mu_transition_proposal, 1.).pdf(data).prod()
        
        # Compute prior probability of current and proposed mu        
        current_prior = norm(prior_mu, prior_std).pdf(mu_current_evidence)
        proposal_prior = norm(prior_mu, prior_std).pdf(mu_transition_proposal)
        
        current_prob = (current_likelihood * current_prior)
        proposal_prob = (proposal_likelihood * proposal_prior)
        print current_prob, proposal_prob
       
       # Accept or reject proposal?
        accept_prob = proposal_prob / current_prob
        
        # Also check if  Usually would include prior probability, which we neglect here for simplicity
        accept = np.random.rand() < accept_prob

        if accept:
            # Update the position of based on high likelihood
            mu_current_evidence = mu_transition_proposal
        
        posterior.append(mu_current_evidence)
        
    return posterior


if __name__ == '__main__':

    MCMC_Metro(data, plot=True)
    cal_post_analytical(data, 20, mu, sig)  # example 
