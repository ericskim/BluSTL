%% Example Freeway Network
% Model and variable names based off the model found in: 
% "Optimal freeway ramp metering using the asymmetric cell transmission
% model", G. Gomes and R. Horowitz.  
% 
% 

%% Defining Network Parameters 

CTM.nSegments = 3;
CTM.has_onramp = [1,1,0];  % 1 = onramp exists on segment
CTM.has_offramp = [1,0,0]; %
CTM.is_metered = [1,1,0]; % 1 = metered, 0 = not metered

% State space
CTM.seg_NMax = [11.5, 11.5, 11.5];
% NOTE: Onramps implicity have infinite occupancy.

% Blending coefficient
CTM.gamma = ones(CTM.nSegments, 1);

% Max flow
CTM.seg_Fbar = [1.66, 1.66, 1.66];

% Freeflow Velocity 
CTM.freeflow_velocity = [0.7241, 0.7241, 0.7241];

% Split ratio from segments to offramps
CTM.beta = [.1, 0,0];
CTM.beta_bar = zeros(size(CTM.beta)) ...
               + (CTM.beta > 0) .* (1 - CTM.beta);

% Onramp metered flow rate ranges
CTM.umin = [.2,.2,0];
CTM.umax = [.8,.8,0];

% Maximum demand to onramps and freeway segment
CTM.segD = [1, 0, 0];
CTM.onrampD = [.4, .4, 0];

% Onramp flow allocation
CTM.Xi = ones(CTM.nSegments,1);

% Congestion Wave Speeds
CTM.w = .181 * ones(CTM.nSegments,1);

% Set objective (if it isn't to maximize satisfaction robustness)
CTM.objective = 'min_ramp_wait';

%% Declare System
Sys = STLC_freeway(CTM);