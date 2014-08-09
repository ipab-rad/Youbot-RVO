clear all

file='log4.csv';

X = 1;
Y = 2;

Param = csvread(file, 0, 0, [0 0 0 2]);
dt = Param(1,1) * 0.1;  % Reduced by factor of 10 speed of Simulation
nAgents = Param(1,2);
ARadius = Param(1,3) * 500;  % Augmented by factor 10 size of Agents

M = csvread(file, 1);
[Ml,Mw] = size(M);
fig1 = figure(1);
Agent = zeros(nAgents, Ml, 2);

for j=1:nAgents
    Agent(j,:,:) = M(:,j*2:1+j*2);
end

for i=1:Ml
    clf(fig1);
    hold on
    for a=1:nAgents
        h=scatter(Agent(a,i,X),Agent(a,i,Y),ARadius,'blue');
    end
    axis([-10 10 -10 10]);
    pause(dt);
    
end