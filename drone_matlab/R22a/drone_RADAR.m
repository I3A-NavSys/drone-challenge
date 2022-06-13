function [] = drone_RADAR( u )

persistent posXold posYold posZold

figName = 'drone RADAR';
figHandler = findobj('Type','figure','Name',figName)';
if (isempty(figHandler))
    
    % la figura no existe, hay que construirla
    figHandler = figure( ...
        'Name',figName, ...
        'Position',[100 100 440 400],...
        'NumberTitle','off', ... %'MenuBar', 'none', ...
        'Resize','on');
    
    % eje dentro de la figura
    axesHandler = axes(      ...
      'Parent', figHandler,  ...
      'Units','normalized', ...
      'Visible','on');
    grid(axesHandler,'on');
    hold(axesHandler,'on');
    axesHandler.XTick = [-5 -4 -3 -2 -1 0 1 2 3 4 5];
    axesHandler.YTick = [-5 -4 -3 -2 -1 0 1 2 3 4 5];
    axesHandler.ZTick =                [0 1 2 3 4 5];
    axis([-5 +5 -5 +5 0 5])
    
    %round1.sh
%     viscircles([-3 -3],0.5,'color',[1 1 1]*0.7)
%     viscircles([ 3  2],0.2,'color','red')
%     viscircles([-2  1],0.2,'color','green')
%     viscircles([ 3 -1],0.2,'color','blue')

%     %escenario1.sh
%     viscircles([ 1  2],0.5,'color',[1 1 1]*0.7)
%     viscircles([-2 -3],0.2,'color','red')
%     viscircles([ 0 -3],0.2,'color','green')
%     viscircles([ 2 -3],0.2,'color','blue')
    
    %escenario2.sh
    viscircles([-3 -3],0.5,'color',[1 1 1]*0.7)
    viscircles([-1  3],0.2,'color','red')
    viscircles([-3  0],0.2,'color','green')
    viscircles([ 3 -2],0.2,'color','blue')
    
end

figure(figHandler)

% Cargo datos
posX  = u(1);
posY  = u(2);
posZ  = u(3);

if isempty(posXold)
    posXold = posX;
    posYold = posY;
    posZold = posZ;
end

%Pintamos la estela del drone
plot3([posXold posX],[posYold posY],[posZold posZ],...
      'lineWidth',2,...
      'Color','black');
  
posXold = posX;
posYold = posY;
posZold = posZ;

end