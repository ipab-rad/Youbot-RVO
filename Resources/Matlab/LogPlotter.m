clear all

file='PersonWalk4.csv';

X = 1;
Y = 2;

minX = -8;
maxX = -2;
minY = 0;
maxY = 6;

Param = csvread(file, 0, 0, [0 0 0 2]);
dt = Param(1,1) * 0.5;  % Reduced by factor of 10 speed of Simulation
nAgents = Param(1,2);
ARadius = Param(1,3) * 500;  % Augmented by factor 10 size of Agents
nGoals = 3;
Goal(1,:) = [-6.2, 1.5];
Goal(2,:) = [-3.3, 1.5];
Goal(3,:) = [-4.45, 3.3];

% offset = 0.25;
offset = 0;

M = csvread(file, 1);
[Ml,Mw] = size(M);

fig2 = subplot(1,2,2);
Agent = zeros(nAgents, Ml, 2);
Vel = zeros(nAgents, Ml, 2);
Rat = zeros(nAgents, Ml, 3);

% Reading file and extracting values
for j=1:nAgents
    Agent(j,:,:) = M(:,j*2:1+j*2);
    Vel(j,:,:) = M(:,j*4:1+j*4);
    Rat(j,:,:) = M(:,j*6:2+j*6);
end
    
%     clf(fig2);
%       bar([0.3, 0.3, 0.4]);
%     p = bar(Rat(1,23,:));
%       goalRat(1:3)=[0.3, 0.3, 0.4];

%     fig1 = 
for i=1:Ml
    subplot(1,2,1);
    plot(NaN);      % Clear subplot
    hold on
    
%     Plotting Goals
    for g=1:nGoals
      f=scatter(Goal(g,X),Goal(g,Y),ARadius,'red','+');
    end

%     Plotting Agent Positions and Velocities
    for a=1:nAgents
      h=scatter(Agent(a,i,X)+offset,Agent(a,i,Y),ARadius,'blue');
      x=[Agent(a,i,X)+offset,Agent(a,i,X)+offset+Vel(a,i,X)];
      y=[Agent(a,i,Y),Agent(a,i,Y)+Vel(a,i,Y)];
      plot(x, y);
      
    end

    axis([minX maxX minY maxY]);
    axis square;

    % Plotting Goal Ratios
    hold off
    subplot(1,2,2);
    goalRat(1:3)=Rat(1,i,:);
    p = bar(goalRat);
    ylim([0 1]);

    pause(dt);
    
    
end