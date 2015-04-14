%% File Setup
close all
clear all

name = 'GoalSampling4';
file = strcat(name,'.csv');
record = false;
ExpTitle = '2 Robots cycling in opposing directions';
VidName = strcat(name,'avi');
showCounterfactuals = false;
showCounterfactualTrajectories = false;
showErrorBars = false;

X = 1;
Y = 2;

% InSpace Environment
minX = -8;
maxX = -2;
minY = 0;
maxY = 6;

% % Atrium Environment 
% minX = -2;
% maxX = 13;
% minY = -2;
% maxY = 11;


Param = csvread(file, 0, 0, [0 0 0 2]);
% dt = Param(1,1) * 0.2;
dt = 0.001; % Default
SimSpeed = Param(1,1);
nPlannerAgents = Param(1,2);
ARadius = Param(1,3) * 600;
vMag = 1;   % Magnitude scalar for representing velocity vectors
AoffX = 0.2;
AoffY = 0.3;
GoffX = -0.07;
GoffY = -0.3;

M = csvread(file, 1);
[Ml,Mw] = size(M);

MaxNAgents=M(Ml,2);
ColorNum = MaxNAgents;
% ColorNum = 8;
% cmap = hsv(ColorNum);
cmap = lines(ColorNum);
% cmap = jet(ColorNum)

figure, set(gcf, 'Renderer', 'painters', 'Color', 'White', 'Position', [300 100 900 800]);

if record
  vidObj = VideoWriter(VidName,'Motion JPEG AVI');
  vidObj.Quality = 100;
  vidObj.FrameRate = 10;
  open(vidObj);
end

TrajHist = 20;
TrajFreq = 1;
TrajPoints = TrajFreq / SimSpeed;
Trajectory = zeros(MaxNAgents,TrajHist,2);
TotTime = M(Ml,1);
% Reading file and extracting values

%% Main Loop
for i=1:Ml
 tic;
  time = M(i,1);
  nAgents = M(i,2);
  nModelled = M(i,3);
  ModelledAgents = M(i,4:4-1+nModelled);
  nGoals = M(i,4+nModelled);
  TrajID = ModelledAgents;

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
    TrajID(a) = ModelledAgents(a) + 1;
    
    if (rem(i,TrajPoints)==1)
        Trajectory(TrajID(a),:,:)=circshift(Trajectory(TrajID(a),:,:),1,2);
        Trajectory(TrajID(a),1,:)=AgentPos(a,:);
    end

    for g=1:nGoals
      SimVels(a,g,:) = M(i, ((a-1)*6)+(6*nModelled)+((g-1)*2)+4+X+nModelled+(nGoals*2):((a-1)*6)+(6*nModelled)+((g-1)*2)+4+Y+nModelled+(nGoals*2));
    end

    Likelihoods(a,:) = M(i, ((a-1)*3)+(6*nModelled)+4+X+nModelled+(nGoals*4)+((nModelled-1)*nGoals*2):((a-1)*3)+(6*nModelled)+4+nGoals+nModelled+(nGoals*4)+((nModelled-1)*nGoals*2));

    Posterior(a,:) = M(i, ((a-1)*3)+(6*nModelled)+4+X+nModelled+(nGoals*4)+(nModelled*nGoals)+((nModelled-1)*nGoals*2):((a-1)*3)+(6*nModelled)+4+nGoals+nModelled+(nGoals*4)+(nModelled*nGoals)+((nModelled-1)*nGoals*2));
  end
  
  
  %% Main Plot
%   subplot(2,nModelled,1);
%   subplot(2,2,3); % Left bottom
%   subplot(2,nModelled,nModelled+1:nModelled+nModelled); % Center
%   h1 = subplot(2,4,1);    % ATRIUM
  h1 = subplot(2,2,1);
%   h1 = subplot(2,3,1);
  plot(NaN);      % Clear subplot
  hold on
  title('Agent Positions and Velocities');  
  
  % Agent Goals
%   f=plot((Goals(1:nGoals,X)/2)-3,Goals(1:nGoals,Y)/2,'x','MarkerSize',ARadius/16,'Color','black');
  f=plot(Goals(1:nGoals,X),Goals(1:nGoals,Y),'x','MarkerSize',ARadius/16,'Color','black');

  for a=1:nModelled
    % Agent Positions
    color = cmap(rem(ModelledAgents(a),ColorNum)+1,:);
    s=plot(AgentPos(a,X),AgentPos(a,Y),'o','MarkerSize',ARadius/16,'Color',color,'LineWidth',2);
    text(AgentPos(a,X)+AoffX,AgentPos(a,Y)+AoffY,num2str(ModelledAgents(a)),'FontWeight','bold','Color',color);
    % Agent Trajectories
    t=scatter(Trajectory(TrajID(a),1:TrajHist,X),Trajectory(TrajID(a),1:TrajHist,Y),linspace(150,1,TrajHist),color,'o');
    % Agent Velocities
    x=[AgentPos(a,X),AgentPos(a,X)+vMag*(AgentVel(a,X))];
    y=[AgentPos(a,Y),AgentPos(a,Y)+vMag*(AgentVel(a,Y))];
    plot(x, y, 'color', color,'LineWidth',2);
  end
  
  % Goal numbers
  for g=1:nGoals
    text(Goals(g,X)+GoffX,Goals(g,Y)+GoffY,num2str(g),'FontWeight','bold');
  end
% 
%   set(gca,'YDir','reverse');  % ATRIUM
  axis([minX maxX minY maxY]);
  axis square;
  
  xl=xlim(h1);
  yl=ylim(h1);
  xPos = xl(1);
  yGap = -1;
  spc = -0.5;
  ExpyPos = yl(1)+yGap;
  TyPos = yl(1)+(2*spc) + yGap;
  TryPos = yl(1)+(3*spc)+ yGap;
  ThyPos = yl(1)+(4*spc)+ yGap;
%   ExpyPos = yl(2)+(3);    % ATRIUM
%   TyPos = yl(2)+(5);      % ATRIUM
  t1 = text(xPos, ExpyPos, sprintf('%s', ExpTitle), 'Parent', h1, 'FontWeight', 'bold');
  t2 = text(xPos, TyPos, sprintf('Time: %0.1f of %0.1f seconds', time, TotTime), 'Parent', h1);
  t3 = text(xPos, TryPos, sprintf('Trajectory marker freq: %d Hz', TrajFreq), 'Parent', h1);
  t4 = text(xPos, ThyPos, sprintf('Trajectory History: %d seconds', TrajHist * SimSpeed), 'Parent', h1);
  set(h1,'LineWidth',2)
%   axis fill;
  hold off

%   tic;
  %% COUNTERFACTUAL SIMULATIONS
  if showCounterfactuals
      for g=1:nGoals

    %     h2 = subplot(6,4,(1+((g+2)*4)));  % ATRIUM
        h2 = subplot(2,3+nGoals,(3+nGoals)+g);
    %     h2 = subplot(2,3,3+g);
        plot(NaN);
        hold on

        % Simulated Agent Goals
        j=plot(Goals(g,X),Goals(g,Y),'x','MarkerSize',ARadius/28,'Color','Black');
        
        for a=1:nModelled
          % Simulated Agent Positions
          color = cmap(rem(ModelledAgents(a),ColorNum)+1,:);
          m=plot(AgentPos(a,X),AgentPos(a,Y),'o','MarkerSize',ARadius/24,'Color',color);
          % Simulated Agent trajectories
          if showCounterfactualTrajectories
            t=scatter(Trajectory(TrajID(a),1:TrajHist,X),Trajectory(TrajID(a),1:TrajHist,Y),linspace(10,1,TrajHist),color,'o');
          end
          %           t=plot(Trajectory(TrajID(a),1:TrajPoints:TrajHist,X),Trajectory(TrajID(a),1:TrajPoints:TrajHist,Y),'.','MarkerSize',5,'Color',cmap(rem(ModelledAgents(a),ColorNum)+1,:));
          % Simulated Agent Velocities
          x=[AgentPos(a,X),AgentPos(a,X)+vMag*(SimVels(a,g,X))];
          y=[AgentPos(a,Y),AgentPos(a,Y)+vMag*(SimVels(a,g,Y))];
          plot(x, y, 'color', color,'LineWidth',2);
        end
        title(sprintf('\\bfGoal:%d', g));  
    %     set(gca,'YDir','reverse');
        axis([minX maxX minY maxY]);
        axis square;
        xl=xlim(h2);
        yl=ylim(h2);
        for a=1:nModelled
          LikxPos = xl(1);
    %       LikyPos = yl(1)-(1*a);
          LikyPos = yl(1)-(1+(1.5)*a);    % ATRIUM
          t = text(LikxPos, LikyPos, sprintf('{\\itL}(a%d,g%d): %0.3f', a, g, Likelihoods(a, g)), 'Parent', h2);
        end
        set(h2,'LineWidth',1.5)
        hold off
      end
  end
  %% INFERENCE DATA
  % Plotting Goal Likelihoods
  col = 1;
  tempModNum=nModelled;
  if (nModelled > 2)
      tempModNum = 2;
  end
  for a=1:nModelled
%   for a=1:tempModNum
    hold off
%     id = a+floor(col);
%     subplot(ceil(nModelled/3),4,id);
%     col = col + 0.34;
    subplot(nModelled,2,2*a);
%     subplot(2,3,a+1);
%     goalRat(1:3)=Posterior(a,:);
    goalRat=zeros(nGoals,1);
    goalRat(1:nGoals)=Posterior(a,:);
    goalRatErr = zeros(nGoals,2);
    
    
    
    
    goalRatErr(:,1) = Likelihoods(a,:);
%     
    goalLikAvg = sum(Likelihoods(a,:))/nGoals;
    for g=1:nGoals
        if (Likelihoods(a,g)>goalLikAvg)
                % Posterior rising
                goalRatErr(g,1) = 0;
                goalRatErr(g,2) = ((Likelihoods(a,g)-goalLikAvg)/goalLikAvg)*Posterior(a,g);
        else
                % Posterior sinking
                goalRatErr(g,1) = ((goalLikAvg-Likelihoods(a,g))/goalLikAvg)*Posterior(a,g);
                goalRatErr(g,2) = 0;
        end
    end
    
%     for g=1:nGoals
%         if (Likelihoods(a,g)>goalLikAvg)
%                 goalRatErr(g,1) = 0;
%                 goalRatErr(g,2) = ((goalLikAvg-Likelihoods(a,g))/goalLikAvg)*Posterior(a,g);
%         else
%                 goalRatErr(g,1) = ((Likelihoods(a,g)-goalLikAvg)/goalLikAvg)*Posterior(a,g);
%                 goalRatErr(g,2) = 0; 
%         end
%     end


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
    
    % Error Bars on Bar Graph
    if showErrorBars
    p = barwitherr(goalRatErr,goalRat);
    else
    p = bar(goalRat,'FaceColor',cmap(rem(ModelledAgents(a),ColorNum)+1,:));
    end
    set(p,'FaceColor',cmap(rem(ModelledAgents(a),ColorNum)+1,:));
    title(sprintf('Agent %d Goal Posteriors', ModelledAgents(a)));
    ylim([0 1]);
    hold on
    plot(xlim,[0.5 0.5],'--','Color','k','LineWidth',0.2);
    hold off
  end  

if record
  writeVideo(vidObj,getframe(gcf));
else
  pause(dt);
end
toc;
end

if record
  close(vidObj);
end
