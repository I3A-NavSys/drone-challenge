function game_monitor_func( u )
  
  % límites del escenario
  minX = -5;
  maxX = 5;
  minY = -5;
  maxY = 5;
  maxZ = 4;
  
  % duración máxima del intento (en segundos)
  max_try_time = 180;
  
  % fichero de almacenamiento y nombre del equipo
  persistent writefile;
  if isempty(writefile)
      writefile = ispc;
  end
  persistent excelfile;
  excelfile = 'game_monitor.xls';
  persistent teamname;

  % reloj de Gazebo
  sim_time = u(1);
  sim_time = sim_time + (u(2) * 10^-9);
  
  % variables del monitor del juego
  % tiempo del intento actual (desde la activación del drone, sin penalizaciones)
  persistent current_try_time;
  % penalizaciones en el intento actual
  persistent penalties_time;
  % tiempo del intento actual (con penalizaciones por salidas del escenario)
  persistent current_try_penalized_time;
  % tiempo del último intento (con penalizaciones)
  persistent last_try_time;
  
  % ¿intento activo?
  persistent trying;
  % inicio del intento actual
  persistent try_start;
  % instante en el que el drone salió del escenario
  persistent drone_out_time;
  
  if isempty(current_try_time)
      current_try_time = 0;
  end
  if isempty(penalties_time)
      penalties_time = 0;
  end
  if isempty(current_try_penalized_time)
      current_try_penalized_time = 0;
  end
  if isempty(last_try_time)
      last_try_time = 0;
  end
  if isempty(drone_out_time)
      drone_out_time = 0;
  end
  
  % comprobamos ubicación del drone con respecto a los límites del escenario
  fuera = (u(3)<minX || u(3)>maxX) || (u(4)<minY || u(4)>maxY) || (u(5)>maxZ);
  if fuera
      output_msg = 'DRONE OUT OF SCENARIO !!!';
  else
      output_msg = '';
  end
  if (drone_out_time)
      % en la última ejecución estaba fuera
      if not(fuera)
          % ahora está dentro
          drone_out_time = 0;
      end
  else
      % en la última ejecución estaba dentro
      if fuera
          % ahora está fuera
          drone_out_time = sim_time;
      end
  end
  
  if isempty(trying)
      % inicializacion de trying y tryStart
      if (u(6)==1)
          % los motores están activos, empezamos a contar
          trying = 1;
          try_start = sim_time;
      else
          trying = 0;
          try_start = 0;
      end
  else
      % comprobamos el estado de los motores y actualizamos el monitor
      if u(6) == 1
          % los motores están activos
          if (trying == 0)
              % acaban de activarse, empezamos a contar
              trying = 1;
              try_start = sim_time;
              last_try_time = current_try_penalized_time;
              current_try_time = 0;
              penalties_time = 0;
              current_try_penalized_time = 0;
          else
              % ya estaban activos, actualizamos los tiempos del intento actual
              current_try_time = sim_time - try_start;
              if (drone_out_time)
                  % si el drone está fuera, hay que añadir la penalización
                  penalty = sim_time - drone_out_time;
                  penalties_time = penalties_time + penalty;
                  drone_out_time = sim_time;
              end
              current_try_penalized_time = current_try_time + penalties_time;
              if current_try_time > max_try_time
                 output_msg = 'TIME OVER: MOTORS OFF !!!'; 
                 % enviar mensaje de parada de motores (pendiente de revisión)               
%                  msgtype = rostype.geometry_msgs_Pose;
%                  pub = rospublisher('/quadcopter/buscommand', msgtype);
%                  msg = rosmessage(msgtype);
%                  msg.Position.X = 0;
%                  send(pub, msg);
              end
          end
      else
          % los motores están apagados
          if trying == 1
              % había un intento activo
              trying = 0;
              if writefile
                  % escribimos los tiempos del intento en disco
                  [~,~,rawData] = xlsread(excelfile);
                  nrows = (size(rawData,1));
                  nrows = nrows + 1;
                  b = num2str(nrows);
                  c = strcat('A', b);
                  trydata = [datestr(datetime(['now'])) teamname current_try_time penalties_time current_try_penalized_time]
                  xlswrite(excelfile, trydata, 'Hoja1', c);
              end
          end
      end
  end
  
  % drone
  v_drone = [-0.2 0.2; 0.2 0.2; 0.2 -0.2; -0.2 -0.2];
  % traslación a las coordenadas X-Y actuales
  v_drone(:,1) = v_drone(:,1) + u(3);
  v_drone(:,2) = v_drone(:,2) + u(4);
  
  % creaci�n de la figura (salida del monitor)
  figName = 'Drone Challenge Game Monitor';
  figNumber = findobj('Type','figure','Name',figName)';

  % construimos la figura (si no exist�a previamente)
  if isempty(figNumber)
      
      position = get(0,'DefaultFigurePosition');
      position(1:2) = [500 100];   % asignamos la posici�n deseada
      position(3:4) = [600 600];   % asignamos el tama�o deseado

      % figura
      figNumber = figure( ...
          'Name',figName, ...
          'NumberTitle','off', ...
          'MenuBar', 'none', ...
          'Position',position, ...
          'Resize','off');
      
      % eje dentro de la figura
      axisNumber = axes( ...
        'Parent', figNumber, ...
        'Units','normalized', ...
        'Position',[0.0800 0.0800 0.9000 0.9000], ...
        'Visible','on');
      axis(axisNumber,[-10 10 -10 10]);
      xlabel('X (m)');
      ylabel('Y (m)');
      grid(axisNumber,'on');
      hold(axisNumber,'on');
      
%       logoimg = imread('logo.jpg');
%       logoimg = imresize(logoimg, 0.01);
%       image(-15,-10, logoimg);
      
      % eje a la derecha para indicar la altura del drone (coord. Z)
%       yyaxis right;
%       ylim([0 10]);
%       yyaxis left;
              
      % reloj de Gazebo
%       clockHandler = text(axisNumber, -13, -7, ...
%          strcat('Simulation time: ', num2str(sim_time,'%6.3f')),...
%          'FontSize',10, ...
%          'FontWeight','bold', ...
%          'FontName','FixedWidth', ...
%          'BackGroundColor',[0.9 0.9 0.9]);     

      if writefile          
          % pedir el nombre del fichero en el que almacenar los datos de los intentos
%           excelfile = char(inputdlg('Excel file', 'Game Monitor', 1));
          % crearlo si no existe
          if not(exist(excelfile, 'file'))
              % crear el fichero y a�adir una fila de t�tulo
              titulo = {'Try date','Team name','Try time','Penalties','Total try time'};
              xlswrite(excelfile, titulo, 'Hoja1');
          end
      end
      
      % nombre del equipo que va a realizar los intentos
      teamname = inputdlg('Team name', 'Game Monitor', 1);
      teamstr = strcat('Team name: ',teamname);
      teamHandler = text(axisNumber, -9, 9, ...
         teamstr, ...
         'FontSize',15, ...
         'FontWeight','bold', ...
         'FontName','FixedWidth', ...
         'Color','black', ...
         'BackGroundColor',[0.9 0.9 0.9]);               
     
      % cuadro de texto para avisos importantes
      msgHandler = text(axisNumber, -9, 7, ...
         output_msg, ...
         'FontSize',15, ...
         'FontWeight','bold', ...
         'FontName','FixedWidth', ...
         'Color','red', ...
         'BackGroundColor',[0.9 0.9 0.9]);
     
      % altura del drone
      heightHandler = text(axisNumber, 6, 0, ...
         horzcat('Z (m): ', num2str(u(5),'%4.2f')),...
         'FontSize',10, ...
         'FontWeight','bold', ...
         'FontName','FixedWidth', ...
         'Color','black', ...
         'BackGroundColor',[0.9 0.9 0.9]);

      % tiempo del intento actual
      currenttryHandler = text(axisNumber, -9, -7, ...
         horzcat('Try time: ', num2str(current_try_time,'%6.3f'), ' s'),...
         'FontSize',10, ...
         'FontWeight','bold', ...
         'FontName','FixedWidth', ...
         'Color','blue', ...
         'BackGroundColor',[0.9 0.9 0.9]);
     
      % penalizaciones
      penaltiesHandler = text(axisNumber, -9, -8, ...
         horzcat('Penalties: ', num2str(penalties_time,'%6.3f'), ' s'),...
         'FontSize',10, ...
         'FontWeight','bold', ...
         'FontName','FixedWidth', ...
         'Color','red', ...
         'BackGroundColor',[0.9 0.9 0.9]);
     
      % tiempo del intento actual (con penalizaciones)
      currenttrypenalizedHandler = text(axisNumber, -9, -9, ...
         horzcat('Total try time: ', num2str(current_try_penalized_time,'%5.2f'), ' s'),...
         'FontSize',10, ...
         'FontWeight','bold', ...
         'FontName','FixedWidth', ...
         'Color','black', ...
         'BackGroundColor',[0.9 0.9 0.9]);
     
      % tiempo del último intento
%       lasttryHandler = text(axisNumber, -13, -13.5, ...
%          horzcat('Last try time (including penalties): ', num2str(last_try_time,'%6.3f'), ' s'),...
%          'FontSize',10, ...
%          'FontWeight','bold', ...
%          'FontName','FixedWidth', ...
%          'Color','black', ...
%          'BackGroundColor',[0.9 0.9 0.9]);               
      
      % suelo (gold)
      boxHandler = patch(axisNumber, ...
         'Vertices',[-5 5; 5 5; 5 -5; -5 -5], ...
         'Faces',[1 2 3 4], ...
         'FaceColor',[0.9  0.75 0], ...
         'LineStyle', 'none');

      % drone
      droneHandler = patch(axisNumber, ...
         'Vertices',v_drone, ...
         'Faces',[1 2 3 4], ...
         'FaceColor','green');
       
      %Guardamos los manejadores de los objetos a refrescar
      set(axisNumber,'UserData',[ ...
          teamHandler, ...
          msgHandler, ...
          heightHandler, ...
          currenttryHandler, ...
          penaltiesHandler, ...
          currenttrypenalizedHandler, ...
          boxHandler, ...
          droneHandler]);       

  end  
   
  %Recuperamos los manejadores de los objetos de la figura
  axisNumber = get(figNumber, 'Children');
  handles = get(axisNumber,'UserData');
  teamHandler = handles(1);
  msgHandler = handles(2);
  heightHandler = handles(3);
  currenttryHandler = handles(4);
  penaltiesHandler = handles(5);
  currenttrypenalizedHandler = handles(6);
  boxHandler = handles(7);
  droneHandler = handles(8);
  
  %Actualiza objetos
%   set(clockHandler, 'String', strcat('Simulation time: ', num2str(sim_time,'%6.3f')));
  set(msgHandler, 'String', output_msg); 
  if u(5) > maxZ
      set(heightHandler, 'Color', 'red');
  else
      set(heightHandler, 'Color', 'black');
  end
  set(heightHandler, 'String', horzcat('Z (m): ', num2str(u(5),'%4.2f'))); 
  set(currenttryHandler, 'String', horzcat('Try time: ', num2str(current_try_time,'%6.3f'), ' s')); 
  set(penaltiesHandler, 'String', horzcat('Penalties: ', num2str(penalties_time,'%6.3f'), ' s'));
  set(currenttrypenalizedHandler, 'String', horzcat('Total try time: ', num2str(current_try_penalized_time,'%6.3f'), ' s')); 
%   set(lasttryHandler, 'String', horzcat('Last try time (including penalties): ', num2str(last_try_time,'%6.3f'), ' s')); 
  set(droneHandler, 'Vertices', v_drone);
  if fuera
      set(droneHandler, 'Facecolor', 'red');
  else
      set(droneHandler, 'Facecolor', 'green');
  end

  %Redibuja la escena
  drawnow limitrate
 
end


