close all
clear all

file='AtriumExperimentFinal2.csv';
record = true;
ExpTitle = 'Dense navigation in Atrium';
VidName = 'AtriumExperimentFinal2.avi';

X = 1;
Y = 2;

% % InSpace Environment
% minX = -8;
% maxX = -2;
% minY = 0;
% maxY = 6;

% Atrium Environment 
minX = -2;
maxX = 13;
minY = -2;
maxY = 11;

Param = csvread(file, 0, 0, [0 0 0 2]);
% dt = Param(1,1) * 0.2;
dt = 0.025;
nPlannerAgents = Param(1,2);
ARadius = Param(1,3) * 200;
vMag = 1;   % Magnitude scalar for representing velocity vectors

M = csvread(file, 1);
[Ml,Mw] = size(M);

ColorNum = M(Ml,2);
cmap = hsv(ColorNum);
% cmap = lines(ColorNum);
% cmap = jet(ColorNum);

figure, set(gcf, 'Color', 'White', 'Position', [300 100 900 800]);


if record
  % aviobj = avifile('sample.avi','compression','None');
%   nFrames = 10;
  vidObj = VideoWriter(VidName,'Motion JPEG AVI');
  vidObj.Quality = 100;
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
  h1 = subplot(2,4,1);
%   h1 = subplot(2,2,1);
  plot(NaN);      % Clear subplot
  hold on
  title('Distributed Tracker Data');  
  
%     Plotting Goals
  for g=1:nGoals
    f=scatter(Goals(g,X),Goals(g,Y),ARadius,'black','+','LineWidth',1);
  end

%     Plotting Agent Positions and Velocities
  for a=1:nModelled
    s=scatter(AgentPos(a,X),AgentPos(a,Y),ARadius,cmap(rem(ModelledAgents(a),ColorNum)+1,:),'LineWidth',2);
    x=[AgentPos(a,X),AgentPos(a,X)+vMag*(AgentVel(a,X))];
    y=[AgentPos(a,Y),AgentPos(a,Y)+vMag*(AgentVel(a,Y))];
    plot(x, y, 'color', cmap(rem(ModelledAgents(a),ColorNum)+1,:),'LineWidth',2);
  end
% 
  set(gca,'YDir','reverse');
  axis([minX maxX minY maxY]);
  axis square;
  
  xl=xlim(h1);
  yl=ylim(h1);
  xPos = xl(1);
%   ExpyPos = yl(1)-(1);
%   TyPos = yl(1)-(2);
  ExpyPos = yl(2)+(3);
  TyPos = yl(2)+(5);
  t1 = text(xPos, ExpyPos, sprintf('%s', ExpTitle), 'Parent', h1);
  t2 = text(xPos, TyPos, sprintf('Time:%0.1f seconds', time), 'Parent', h1);
  set(h1,'LineWidth',2)
%   axis fill;
hold off

% Plot counterfactual simulated velocities for each agent
  for g=1:nGoals
    h2 = subplot(6,4,(1+((g+2)*4)));
%     h2 = subplot(2,3+nGoals,(3+nGoals)+g);
    plot(NaN); 
    %     Plotting Goals
    hold on
    j=scatter(Goals(g,X),Goals(g,Y),ARadius/4,'black','+','LineWidth',1);
    for a=1:nModelled
      m=scatter(AgentPos(a,X),AgentPos(a,Y),ARadius/4,cmap(rem(ModelledAgents(a),ColorNum)+1,:),'LineWidth',1.5);
      x=[AgentPos(a,X),AgentPos(a,X)+vMag*(SimVels(a,g,X))];
      y=[AgentPos(a,Y),AgentPos(a,Y)+vMag*(SimVels(a,g,Y))];
      plot(x, y, 'color', cmap(rem(ModelledAgents(a),ColorNum)+1,:),'LineWidth',2);
    end
    title(sprintf('\\bfGoal:%d', g));  
    set(gca,'YDir','reverse');
    axis([minX maxX minY maxY]);
    axis square;
    xl=xlim(h2);
    yl=ylim(h2);
    for a=1:nModelled
      LikxPos = xl(1);
%       LikyPos = yl(1)-(1+(2.5)*a);
      LikyPos = yl(1)-(1+(1.5)*a);
%       t = text(LikxPos, LikyPos, sprintf('{\\itL}(a%d,g%d): %0.3f', a, g, Likelihoods(a, g)), 'Parent', h2);
    end
    set(h2,'LineWidth',1.5)
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

  % Plotting Goal Likelihoods
  col = 1;
  for a=1:nModelled
    hold off
    id = a+floor(col);
    subplot(ceil(nModelled/3),4,id);
    col = col + 0.34;
%     subplot(nModelled,2,2*a);
    %   y = randn(3,4);         % random y values (3 groups of 4 parameters)
    %   errY = zeros(3,4,2);
    %   errY(:,:,1) = 0.1.*y;   % 10% lower error
    %   errY(:,:,2) = 0.2.*y;   % 20% upper error
    %   barwitherr(errY, y);    % Plot with errorbars
%     goalRat(1:3)=Posterior(a,:);
%     p = bar(goalRat,'FaceColor',cmap(rem(ModelledAgents(a),ColorNum)+1,:));
    goalRat=zeros(3,1);
    goalRat(1:3)=Posterior(a,:);
    goalRatErr = zeros(3,2);
%     goalRatErr(:,1) = Likelihoods(a,:);
    
    goalLikAvg = sum(Likelihoods(a,:))/nGoals;
    for g=1:nGoals
        if (Likelihoods(a,g)>goalLikAvg)
                goalRatErr(g,1) = 0;
                goalRatErr(g,2) = ((goalLikAvg-Likelihoods(a,g))/goalLikAvg)*Posterior(a,g);
        else
                goalRatErr(g,1) = ((Likelihoods(a,g)-goalLikAvg)/goalLikAvg)*Posterior(a,g);
                goalRatErr(g,2) = 0; 
        end
    end

%   Real Likelihoods
%     for g=1:nGoals
%         if (Likelihoods(a,g)>Posterior(a,g))
%                 goalRatErr(g,1) = 0;
%                 goalRatErr(g,2) = ((Posterior(a,g)-Likelihoods(a,g))/Likelihoods(a,g))*Posterior(a,g);
%         else
%                 goalRatErr(g,1) = ((Likelihoods(a,g)-Posterior(a,g))/Posterior(a,g))*Posterior(a,g);
%                 goalRatErr(g,2) = 0; 
%         end
%     end
    
    p = barwitherr(goalRatErr,goalRat);
    set(p,'FaceColor',cmap(rem(ModelledAgents(a),ColorNum)+1,:));
    title(sprintf('Agent %d Goal Posteriors', ModelledAgents(a)));
    ylim([0 1]);
    hold on
    plot(xlim,[0.5 0.5],'--','Color','k','LineWidth',0.2);
    hold off
    %   y = randn(3,4);         % random y values (3 groups of 4 parameters)
%   errY = zeros(3,4,2);
%   errY(:,:,1) = 0.1.*y;   % 10% lower error
%   errY(:,:,2) = 0.2.*y;   % 20% upper error
%   barwitherr(errY, y);    % Plot with errorbars
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
