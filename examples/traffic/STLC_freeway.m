classdef STLC_freeway
    % STLC_freeway for cell transmission model 
    % 
    
    % system properties
    properties
        sys      % system as control toolbox objec
        sysd
        nx
        nd
        x0
        system_data
        nSegments
        has_onramp
        has_offramp
        is_metered
        seg_NMax
        gamma
        seg_Fbar
        freeflow_velocity
        beta
        beta_bar
        segD
        onrampD
        Xi
        w
        
        ramp2X
        nOnramps
        
    end
    
    % controller properties
    properties
        umin
        umax
        u_delta
        ts         % sampling time
        L          % horizon
        time
        nb_stages
        var
        stl_list
        encoding  % specifies the technique for encoding TODO
        min_rob    % TODO: if rob==0 use non robust encoding
        lambda_rho %  weight of robustness in the cost function
        bigM
        solver_options
        model_data
        controller    % YALMIP parametric problem for the adversary

    end
    
    % adversary properties
    properties
        Wref         % this defines a default or initial disturbance vector
        w_lb         % lower bound on w relative to Wref
        w_ub         % upper bound on w relative to Wref
        stl_w_list   % stl properties for environment
        max_react_iter % maximum number of iterations
        adversary    % YALMIP parametric problem for the adversary
    end
        
    % plotting properties
    properties
        h
        xlabel  % labels (names) for x signals
        %ylabel
        %ulabel
        dlabel
        plot_x  % indices of states to plot
        %plot_y  % indices of outputs to plot
        %plot_u  % index of inputs to plot
        plot_d  % index of disturbances to plot
    end
    
    % misc
    properties
       stop_button 
       verbosity 
    end
    
    methods
        function Sys = STLC_freeway(model)
            
            Sys.nSegments = model.nSegments;
            Sys.has_onramp = model.has_onramp;
            Sys.nOnramps = size(find(Sys.has_onramp));
            
            Sys.ramp2X = zeros(size(Sys.nSegments));
            for ramp = find(Sys.has_onramp)
                Sys.ramp2X(ramp) = Sys.nSegments + ramp;
            end
            
            Sys.has_offramp = model.has_offramp;
            Sys.nOfframps = size(find(Sys.has_offramp));
            
            Sys.is_metered = model.is_metered;
            Sys.seg_NMax = model.segNMax;
            Sys.gamma = model.gamma;
            Sys.seg_Fbar = model.seg_Fbar;
            Sys.freeflow_velocity = model.freeflow_velocity;
            Sys.beta = model.beta;
            Sys.beta_bar = model.beta_bar;
            Sys.segD = model.segD;
            Sys.onrampD = model.onrampD;
            Sys.Xi = model.Xi;
            Sys.w = model.w;
            Sys.umin = model.umin;
            Sys.umax = model.umax;
            
            Sys.nu = size(find(Sys.has_onramp));

            Sys.x0 = zeros(Sys.nSegments + size(find(Sys.has_onramp)),1);
            
            % default options
            Sys.solver_options = sdpsettings('solver','gurobi','verbose',1, 'cachesolvers',1);
            Sys.min_rob = 0.01;
            Sys.lambda_rho = 0;
            Sys.bigM = 1000;
            Sys.u_delta = Inf;
            Sys.max_react_iter = 10;
            Sys.nb_stages = 1;
            Sys.stop_button = 0;
            
            
            
            % default values for input constraints - note, forces u to 0
            %Sys.u_lb = zeros(1,Sys.nu);
            %Sys.u_ub = zeros(1,Sys.nu);
            %Sys.u_delta = Inf*ones(1,);
            
            % default values for disturbance constraints - note:w_lb and w_ub are relative to Wref
            % i.e.,   wref+w_lb  <= w <= wref + w_ub. Thus by default, w == Wref
            %Sys.w_lb = zeros(1,Sys.nw);
            %Sys.w_ub = zeros(1,Sys.nw);
            
        end
        
        % TODO continuous-time for system step, interface to other (NL,
        % Simulink, external) dynamics)
%         function [x1, y0] = system_step(Sys, x0, u0, w0)
%             x1 = Sys.sysd.A*x0+ Sys.sysd.B*[u0; w0];
%             y0 = Sys.sysd.C*x0+ Sys.sysd.D*[u0; w0];
%         end
               
        % default objective function r is the robust sat. and wr a weight 
        function obj = get_objective(Sys, X, Y, U,W, rho,wr)
            switch nargin
                case 4
                    obj = sum(sum(abs(U))); % minimize U
                case 6
                    obj = sum(sum(abs(U)))-sum(rho); % minimize U penalized by r 
                case 7
                    obj = sum(sum(abs(U)))-wr*sum(rho);
            end
        end
      
        function controller = get_controller(Sys,enc)
            if nargin < 2
                enc = 'robust';
            end
            Sys.sysd = c2d(Sys.sys, Sys.ts);
            controller = STLC_get_controller(Sys,enc);
        end
        
        function adversary = get_adversary(Sys)
            Sys.sysd = c2d(Sys.sys, Sys.ts);
            adversary = STLC_get_adversary(Sys);
        end
        
        % Executes the controller in open loop mode
        function [system_data, params] = run_open_loop(Sys, controller)
            [system_data, params] = STLC_run_open_loop(Sys, controller);
        end
        
        % Executes the controller in a receding horizon (MPC)
        function [Sys, params] = run_deterministic(Sys, controller)
            [Sys, params] = STLC_run_deterministic(Sys, controller);
        end
        
        % Executes controller and adversary in open loop
        function [Sys, params] = run_open_loop_adv(Sys, controller, adversary)
            [Sys, params] = STLC_run_open_loop_adv(Sys, controller, adversary);
        end
        
        % Executes controller and adversary in receding horizon mode (MPC)
        function [Sys, params] = run_adversarial(Sys, controller, adversary)
            [Sys, params] = STLC_run_adversarial(Sys, controller, adversary);
        end
        
        % Default plot function
        function Sys = update_plot(Sys)
            Sys = STLC_update_plot(Sys);
        end
    end
end