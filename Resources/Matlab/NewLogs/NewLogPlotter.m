close all
clear all

file='1P2Youbots-F5.csv';
record = false;

X = 1;
Y = 2;

nMaxAgents = 4;

minX = -8;
maxX = -2;
minY = 0;
maxY = 6;
% 
% minX = 0;
% maxX = 13;
% minY = 0;
% maxY = 11;

Param = csvread(file, 0, 0, [0 0 0 2]);
dt = Param(1,1) * 0.1;
nPlannerAgents = Param(1,2);
ARadius = Param(1,3) * 300;

M = csvread(file, 1);
[Ml,Mw] = size(M);

% aviobj = avifile('sample.avi','compression','None');
vidObj = VideoWriter('sample.avi','Motion JPEG AVI');
vidObj.Quality = 80;
open(vidObj);
% Reading file and extracting values
for i=1:Ml
  time = M(i,1);
  nAgents = M(i,2);
  nModelled = M(i,3);
  cmap = hsv(6);
  ModelledAgents = M(i,4:4-1+nModelled);
  nGoals = M(i,4+nModelled);

  Goals = zeros(nGoals, 2);
  AgentPos = zeros(nModelled, 2);
  AgentVel = zeros(nModelled, 2);
  PrefSpeed = zeros(nModelled, 1);
  MaxSpeed = zeros(nModelled, 1);
  SimVels = zeros(nModelled, nGoals, 2);
  Likelihoods = zeros(nModelled, nGoals);
  Posterior = zeros(nModelled, nGoals);
  for g=1:nGoals
    Goals(g,:) = M(i,((g-1)*2)+4+X+nModelled:((g-1)*2)+4+Y+nModelled);
  end
  
  for a=1:nModelled
    AgentPos(a,:) = M(i, ((a-1)*2)+4+X+nModelled+(nGoals*2):((a-1)*2)+4+Y+nModelled+(nGoals*2));
    AgentVel(a,:) = M(i, ((a-1)*2)+(2*nModelled)+4+X+nModelled+(nGoals*2):((a-1)*2)+(2*nModelled)+4+Y+nModelled+(nGoals*2));
    PrefSpeed(a) = M(i, ((a-1)*2)+(4*nModelled)+4+X+nModelled+(nGoals*2));
    MaxSpeed(a) = M(i, ((a-1)*2)+(4*nModelled)+4+Y+nModelled+(nGoals*2));
    
    for g=1:nGoals
      SimVels(a,g,:) = M(i, ((a-1)*6)+(6*nModelled)+((g-1)*2)+4+X+nModelled+(nGoals*2):((a-1)*6)+(6*nModelled)+((g-1)*2)+4+Y+nModelled+(nGoals*2));
    end
    
      Likelihoods(a,:) = M(i, ((a-1)*3)+(6*nModelled)+4+X+nModelled+(nGoals*4)+((nModelled-1)*nGoals*2):((a-1)*3)+(6*nModelled)+4+nGoals+nModelled+(nGoals*4)+((nModelled-1)*nGoals*2));
    
      Posterior(a,:) = M(i, ((a-1)*3)+(6*nModelled)+4+X+nModelled+(nGoals*4)+(nModelled*nGoals)+((nModelled-1)*nGoals*2):((a-1)*3)+(6*nModelled)+4+nGoals+nModelled+(nGoals*4)+(nModelled*nGoals)+((nModelled-1)*nGoals*2));
  end
%   subplot(2,nModelled,1);
  subplot(2,nModelled,nModelled+1:nModelled+nModelled);
  plot(NaN);      % Clear subplot
  hold on

%     Plotting Goals
  for g=1:nGoals
    f=scatter(Goals(g,X),Goals(g,Y),ARadius,'red','+');
  end

%     Plotting Agent Positions and Velocities
  for a=1:nModelled
    h=scatter(AgentPos(a,X),AgentPos(a,Y),ARadius,cmap(ModelledAgents(a)+1,:));
    x=[AgentPos(a,X),AgentPos(a,X)+AgentVel(a,X)];
    y=[AgentPos(a,Y),AgentPos(a,Y)+AgentVel(a,Y)];
    plot(x, y, 'color', cmap(ModelledAgents(a)+1,:));
  end
% 
  
  axis([minX maxX minY maxY]);
  axis square;
 
% 
  % Plotting Goal Likelihoods
  
  for a=1:nModelled
    hold off
    subplot(2,nModelled,a);
    goalRat(1:3)=Posterior(a,:);
    p = bar(goalRat,'FaceColor',cmap(ModelledAgents(a)+1,:));
    ylim([0 1]);
    title(sprintf('Agent %d', ModelledAgents(a)));
  end
  currFrame = getframe(gcf);
  
%   for fr=1:5
%     writeVideo(vidObj,currFrame);
%   end
if record
  writeVideo(vidObj,currFrame);
  writeVideo(vidObj,currFrame);
  writeVideo(vidObj,currFrame);
  writeVideo(vidObj,currFrame);
  writeVideo(vidObj,currFrame);
else
  pause(dt);
end
  
end

close(vidObj);

% 
% fig2 = subplot(1,2,2);
% Agent = zeros(nAgents, Ml, 2);
% Vel = zeros(nAgents, Ml, 2);
% Rat = zeros(nAgents, Ml, 3);
% 
% % Reading file and extracting values
% for j=1:nAgents
%     Agent(j,:,:) = M(:,j*2:1+j*2);
%     Vel(j,:,:) = M(:,j*4:1+j*4);
%     Rat(j,:,:) = M(:,j*6:2+j*6);
% end
%     
% %     clf(fig2);
% %       bar([0.3, 0.3, 0.4]);
% %     p = bar(Rat(1,23,:));
% %       goalRat(1:3)=[0.3, 0.3, 0.4];
% 
% %     fig1 = 
% goalLine1=Rat(1,:,1);
% goalLine2=Rat(1,:,2);
% goalLine3=Rat(1,:,3);
% 
% for i=1:Ml
%     subplot(1,2,1);
%     plot(NaN);      % Clear subplot
%     hold on
%     
% %     Plotting Goals
%     for g=1:nGoals
%       f=scatter(Goal(g,X),Goal(g,Y),ARadius,'red','+');
%     end
% 
% %     Plotting Agent Positions and Velocities
%     for a=1:nAgents
%       h=scatter(Agent(a,i,X)+offset,Agent(a,i,Y),ARadius,'blue');
%       x=[Agent(a,i,X)+offset,Agent(a,i,X)+offset+Vel(a,i,X)];
%       y=[Agent(a,i,Y),Agent(a,i,Y)+Vel(a,i,Y)];
%       plot(x, y);
%       
%     end
% 
%     axis([minX maxX minY maxY]);
%     axis square;
% 
%     % Plotting Goal Ratios
%     hold off
%     subplot(1,2,2);
%     goalRat(1:3)=Rat(1,i,:);
%     p = bar(goalRat);
%     ylim([0 1]);
% 
%     pause(dt);
%     
%     
% end

