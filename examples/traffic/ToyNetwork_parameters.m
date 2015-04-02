% Network Data
clear all;
clc;
% Time Step
sim_dt = 3; % second
% Gamma in ACTM
gamma = 1;
% Split Ratios
beta1 = 0.1;
beta2 = 0; % Sections without off-ramps have split ratio = 0
beta3 = 0;
% Demands
d1 = 0.5 * sim_dt; % Upstream demand
d2 = 0.5 * sim_dt; % Demand of first on-ramp
d3 = 0.5 * sim_dt; % Demand of second on-ramp

beta1_bar = 1-beta1;
beta2_bar = 1-beta2;
beta3_bar = 1-beta3;
% Free Flow Speeds
v1 = 0.24135 * sim_dt;
v2 = 0.24135 * sim_dt;
v3 = 0.24135 * sim_dt;
% Congestion wave speeds
w1 = 0.06034 * sim_dt;
w2 = 0.06034 * sim_dt;
w3 = 0.06034 * sim_dt;
% Jam densities
n1_jam = 11.51003; % main-line jam densities
n2_jam = 11.51003;
n3_jam = 11.51003;
l1_jam = inf; % On-ramp maximum queues
l2_jam = inf;
% Capacities (Maximum Flows)
f1_bar = 0.5556 * sim_dt;
f2_bar = 0.5556 * sim_dt;
f3_bar = 0.5556 * sim_dt;
r1_bar = 0.5556 * sim_dt;
r2_bar = 0.5556 * sim_dt;
% Xi 
Xi = 1;