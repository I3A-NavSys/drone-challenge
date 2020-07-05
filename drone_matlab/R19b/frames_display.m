function [] = frames_display( u )

%% creacion de la figura


%definicion de rectangulos
persistent red
persistent green
persistent blue

figName = 'drone camera';
figHandler = findobj('Type','figure','Name',figName)';
if (isempty(red) || isempty(figHandler))
    
    if isempty(figHandler) 
        % la figura no existe, hay que construirla
        figHandler = figure( ...
            'Name',figName, ...
            'NumberTitle','off', ...
            'MenuBar', 'none', ...
            'Resize','on');
    end
    
    % eje dentro de la figura
    axesHandler = axes(      ...
      'Parent', figHandler,  ...
      'Units','normalized', ...
      'Visible','on');
    grid(axesHandler,'on');
    hold(axesHandler,'on');
    axis equal
    axis([-160 160 -120 120])

    %rectangulos
    red   = rectangle(axesHandler,...
              'Position', [-150,0,50,50],...
              'LineStyle', 'none',...
              'FaceColor', 'red' );
    green = rectangle(axesHandler,...
              'Position', [ -50,0,50,50],...
              'LineStyle', 'none',...
              'FaceColor', 'green' );
    blue  = rectangle(axesHandler,...
              'Position', [  50,0,50,50],...
              'LineStyle', 'none',...
              'FaceColor', 'blue' );
end



%% Actualizacion de rectangulos

if(u(1) == -inf)
    red.Visible = 'off';
else
    red.Position =   [u(4) u(2) u(3)-u(4) u(1)-u(2)];
    red.Visible = 'on';
end


if(u(7) == -inf)
    green.Visible = 'off';
else
    green.Position = [u(10) u(8) u(9)-u(10) u(7)-u(8)];
    green.Visible = 'on';
end


if(u(13) == -inf)
    blue.Visible = 'off';
else
    blue.Position =  [u(16) u(14) u(15)-u(16) u(13)-u(14)];
    blue.Visible = 'on';
end
  
