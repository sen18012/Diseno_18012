%Universidad del Valle de Guatemala                  Katharine Senn Salazar
%Diseño e Innovación en Ingeniería                              Carné 18012            
%Sección 20
%
%                              MINI PROYECTO 1
% 
%_______________________________________________________________________

%x = (atan2(fAccel[0], sqrt (fAccel[1] * fAccel[1] + fAccel[2] * fAccel[2]))*180.0)/3.14;
%y = (atan2(fAccel[1], sqrt (fAccel[0] * fAccel[0] + fAccel[2] * fAccel[2]))*180.0)/3.14;
%z = (atan2(fAccel[0], fAccel[1])*180.0)/3.14;

close all;
clear
N = 1000;
t = linspace(0,40,N)';
K = 10;
%info = zeros(T,1);

delete(instrfind);
%Se configura el puerto en el que esta conectado la tivaC
%Y el baudrate al que se operara para la comunicación
ser = serialport('COM6',115200);
figure(7); clf;
h7 = plot(t,zeros(N,1));
xlim([0,t(end)]);
ylim([-90,90]);
buffer = zeros(K,1);
k = 1;
%fopen(s);
%Ciclo for

for n = 1:N
    %%Se leen los datos y se almacenan en el vector
    %%y(n,1) = str2double(readline(ser))
    data = readline(ser);
    data_split = split(data," ");
    giro_x = data_split(1);
    giro_y = data_split(2);
    giro_z = data_split(3);
    acel_x = data_split(4);
    acel_y = data_split(5);
    acel_z = data_split(6);
    angu_z = data_split(7);
    
%     acel_x1 = str2double(acel_x);
%     acel_y1 = str2double(acel_y);
%     angulo = (atan2(acel_x1, acel_y1)*180.0)/3.14;
   
    y(n,1) = str2double(angu_z)-90;
    drawnow limitrate
    
       buffer(k) = y(n);
    
    if(k == K)
        h7.YData((n-K+1):n) = buffer;   % Asume que K es factor de N. De lo contrario,
                                        % hay que hacer ajustes adicionales.
        drawnow limitrate
        k = 1;
    else
        k = k + 1;    
    end
end
%fclose(s);

