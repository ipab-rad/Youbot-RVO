close all
clear all

file='2p3PYoubots-C4-RERUN2.csv';
record = false;

cmap = hsv(15);

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
% dt = Param(1,1) * 0.2;
dt = 0.025;
nPlannerAgents = Param(1,2);
ARadius = Param(1,3) * 300;

M = csvread(file, 1);
[Ml,Mw] = size(M);

figure, set(gcf, 'Color','white', 'Position', [680 678 800 600]);


if record
  % aviobj = avifile('sample.avi','compression','None');
%   nFrames = 10;
  vidObj = VideoWriter('Test5.avi','Motion JPEG AVI');
  vidObj.Quality = 80;
  vidObj.FrameRate = 10;
  open(vidObj);

  
  %# preallocate
%   nFrames = 10;
%   mov(1:Ml) = struct('cdata',[], 'colormap',[]);
%   k = 1;
%   
%   set(gca, 'nextplot','replacechildren', 'Visible','off');
end
% Reading file and extracting values
for i=1:Ml
  time = M(i,1);
  nAgents = M(i,2);
  nModelled = M(i,3);
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
%   subplot(2,2,3); % Left bottom
%   subplot(2,nModelled,nModelled+1:nModelled+nModelled); % Center
  subplot(2,2,1);
  plot(NaN);      % Clear subplot
  hold on
  title('Tracked Data');  
  
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
%   axis fill;
hold off

% Plot counterfactual simulated velocities for each agent
  for g=1:nGoals
    h2 = subplot(2,3+nGoals,(3+nGoals)+g);
    plot(NaN); 
    %     Plotting Goals
    hold on
    j=scatter(Goals(g,X),Goals(g,Y),ARadius,'red','+');
    for a=1:nModelled
      m=scatter(AgentPos(a,X),AgentPos(a,Y),ARadius,cmap(ModelledAgents(a)+1,:));
      x=[AgentPos(a,X),AgentPos(a,X)+SimVels(a,g,X)];
      y=[AgentPos(a,Y),AgentPos(a,Y)+SimVels(a,g,Y)];
      plot(x, y, 'color', cmap(ModelledAgents(a)+1,:));
    end
    title('Sim:%d', g);  
    axis([minX maxX minY maxY]);
    axis square;
    xl=xlim(h2);
    yl=ylim(h2);
    for a=1:nModelled
      LikxPos = xl(1)+2;
      LikyPos = yl(1)-(1+a); 
      t = text(LikxPos, LikyPos, sprintf('A%d:%f', a, Likelihoods(a, g)), 'Parent', h2);
      set(t, 'HorizontalAlignment', 'center');
    end
      hold off
  end
%   
%   for g=1:nGoals
%     
% %     hold on
% %     ax = subplot(3,3+nGoals,(3+nGoals)+g+2*nModelled);
%       for a=1:nModelled
%         ax = subplot(4,3+nGoals,3*(3+nGoals)+g);
%         text(-0.1,-0.1,'Hi');
%         set ( ax, 'visible', 'off')
%       end
%     hold off
%   end

  

  
 
% 
  % Plotting Goal Likelihoods
  for a=1:nModelled
    hold off
    subplot(nModelled,2,2*a);
    goalRat(1:3)=Posterior(a,:);
    p = bar(goalRat,'FaceColor',cmap(ModelledAgents(a)+1,:));
    ylim([0 1]);
    title(sprintf('Agent %d', ModelledAgents(a)));
  end  
  

  
%   for fr=1:5
%     writeVideo(vidObj,currFrame);
%   end
if record
%   mov(k) = getframe(gcf);
%   k = k + 1;
  writeVideo(vidObj,getframe(gcf));
%   writeVideo(vidObj,getframe(gca));
%   writeVideo(vidObj,getframe(gca));
%   writeVideo(vidObj,getframe(gca));
%   writeVideo(vidObj,currFrame);
else
  pause(dt);
end
  
end

if record
  close(vidObj);
% close(gcf);
end

% movie2avi(mov, 'Test4.avi', 'compression','None', 'fps', 10);
