%% Defining Network Topology

Freeway.nLinks = 5;
Freeway.nSigs = 3;

%% State space
Freeway.xMax = [80, 25, 80, 25, 80];

%% Max flow
Freeway.c = [30,10,30,10,30];

%% Define demand parameters
Freeway.beta = zeros(Freeway.nLinks);
Freeway.beta(1,3) = .8;
Freeway.beta(3,5) = .8;
Freeway.beta(2,3) = 1;
Freeway.beta(4,5) = 1;

%% Define control parameters
% For each signal
Freeway.U = cell(Freeway.nSigs,1);
Freeway.U{1} = {{1}; {1,2}};
Freeway.U{2} = {{3}; {3,4}};
Freeway.U{3} = {{5}};

% Figure out how many total control actions there are
Freeway.numU = 1;
Freeway.config_per_sig = zeros(Freeway.nSigs,1);
for i = 1:size(Freeway.U,1) % iterate over signals
    Freeway.numU = Freeway.numU * size(Freeway.U{i},1);
    Freeway.config_per_sig(i) = size(Freeway.U{i},1);
end

%% Find downstream vertices
Freeway.down_vertex = cell(Freeway.nLinks,1);
Freeway.up_vertex = cell(Freeway.nLinks,1);

for sig = 1:Freeway.nSigs
    for sigConfig = 1:size(Freeway.U{sig},1) %iterate over signal configurations
        for link = cell2mat(Freeway.U{sig}{sigConfig})
            Freeway.down_vertex{link} = sig;
        end
    end
end

%% Find upstream and downstream links 
Freeway.down_links = cell(Freeway.nLinks,1);
Freeway.up_links = cell(Freeway.nLinks,1); 

for link = 1:Freeway.nLinks
    Freeway.down_links{link} = find(Freeway.beta(link,:));
    Freeway.up_links{link} = find(Freeway.beta(:,link));
end

%% Find upstream vertices
for link = 1:Freeway.nLinks
    if ~isempty(Freeway.up_links{link})
        Freeway.up_vertex{link} =  Freeway.down_vertex{Freeway.up_links{link}(1)};
    end
end

%% Define supply parameters 
Freeway.alpha = cell(Freeway.nLinks,1);

% Define initial supply data structure
for link = 1:Freeway.nLinks
    if ~isempty(Freeway.up_links{link})
        Freeway.alpha{link} = cell(Freeway.config_per_sig(Freeway.up_vertex{link}),1);
    end
end

Freeway.alpha{3}{1} = [1.0, 0, 0, 0, 0];
Freeway.alpha{3}{2} = [.7, .3, 0, 0, 0];
Freeway.alpha{5}{1} = [0, 0, 1.0, 0, 0];
Freeway.alpha{5}{2} = [0, 0, .7, .3, 0];

%% Disturbance sets
Freeway.d = [15,2.5, 0, 2.5,0];

%% Declare System
Sys = STLC_traffic(Freeway);