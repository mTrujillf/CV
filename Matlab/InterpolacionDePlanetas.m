
disp('                   Modelo de Planetas');
disp('========================================================');
% =========================================================================
%Radios 
rls1 = 696340000+1.496e+11;%radio de la órbita Tierra alrededor del centro de la sol; radio del sol y la distancia de la tierra al sol 
rls2 = 6371000+3.832e+08; %radio de la órbita Luna alrededor del centro de la Tierra; primer numero es el radio de la tierra y el segundo la distancia de la tierra a la luna 
rls3 = 696340000+2.4848E+11; %radio de la órbita MARTE alrededor del centro de la SOL. radio del sol y la distancia de Marte al sol 
rls4=696340000+1.0764E+11; %radio de la órbita VENUS alrededor del centro de la SOL.
rls5=696340000+ 6.0335E+10;%radio de la órbita MERCURIO alrededor del centro de la SOL.
rF = 9377000; %distancia de Fobos a Marte
rD = 23460000;%distancia de Deinos a Marte
rM = 3389500;%radio de Marte
clf
G = 6.67408E-11; %%  m3 kg-1 s-2 - Gravedad


ang_incl_Luna = ( 88.3 ) * pi / 180.0;
%
rLuna=3.832e+08;
%
xLuna = rLuna * cos(ang_incl_Luna);
yLuna = 0;
zLuna = rLuna * sin(ang_incl_Luna);

ang_incl_Fobos = ( 26.03 ) * pi / 180.0;
%
xFobos = rF * cos(ang_incl_Fobos);
yFobos = 0;
zFobos = rF * sin(ang_incl_Fobos);

m         = [1.989E+30 , 5.9736E+24, 7.349E+22, 6.4185E+23, 4.867E+24, 3.285E+23,1.072e+16,2e+15]; %masas del sol, de la tierra, masa de la luna, masa marte, venus, MERCURIO
ndm       = size(m);
n_moviles = ndm(2);
M         = zeros(3,n_moviles);
for i = 1:n_moviles
    M(:,i) = m(i) * ones(3,1); 
    
end

wls1 = sqrt(G*m(1)/rls1^3);
vTgls1=wls1*rls1; %Tierra

wls2 = sqrt(G*m(2)/rLuna^3);
vTgls2=wls2*rLuna;%Luna

wls3 = sqrt(G*m(1)/rls3^3);
vTgls3=wls3*rls3;% Marte

wls4 = sqrt(G*m(1)/rls4^3);
vTgls4=wls4*rls4;%Venus

wls5 = sqrt(G*m(1)/rls5^3);
vTgls5=wls5*rls5;%Mercurio

wls6 = sqrt(G*m(4)/rF^3);
vtgls6 = wls6 * rF; %Fobos

wls7 = sqrt(G*m(4)/rD^3);
vtgls7 = wls7 * rD; %Fobos

%Meto el móvil satélite geo est
r0=[[0;0;0], ...%sol
    [rls1;0;0], ...%tierra
    [rls1+xLuna;yLuna;zLuna],...% luna,...
    [rls3;0;0],...%MARTE
    [rls4;0;0],...%venus
    [rls5;0;0],...%Mercurio
    [xFobos + rls3;yFobos;zFobos],...%Utilizando el calculo de las inclinaciones se coloco  en su posicion inical Fobos                               
    [rD + 2 * rM + rls3;0;0]]; 
    %[3.832557907001450e+08+rls2;0;0], ...
    %[(3.832557907001450e+08);0;rls3]];

v0=[[0;0;0],...
    [0;vTgls1;0],... %...Tierra
    [0;vTgls1+vTgls2;0],...%Luna
    [0;vTgls3;0],...%Marte
    [0;vTgls4;0],...%Venus
    [0;vTgls5;0],...%Mercurio
    [0;vTgls3+vtgls6;0],...
    [0;vTgls3+vtgls7;0]]; %Fobos
    %[0;1.019927755507821e+03+vTgls1;0],...
    %[0;1.019927755507821e+03+vTgls2;0],...
    %[0;0;vTgls3]];
%
deltaT=1*60;  %% cada minuto (en segs)

Nk = 365*24*60*60/deltaT; %en hrs

pos = zeros(Nk+1,3*n_moviles);
for i=1:n_moviles
    pos(1,3*(i-1)+1:3*i)=r0(:,i)';
end

u      = r0;
Fza    = zeros(n_moviles,n_moviles,3); % hacia atrás se guardan [Fx,Fy,Fz]
a      = Fza;
dr     = r0;
FzaTot = zeros(3,n_moviles);

r = r0;
v = v0;

aceleracionX = zeros(Nk,n_moviles);
velocidadX = zeros(Nk,n_moviles);
aceleracionY = zeros(Nk,n_moviles);
velocidadY = zeros(Nk,n_moviles);
aceleracionZ = zeros(Nk,n_moviles);
velocidadZ = zeros(Nk,n_moviles);
aceleracionRes = zeros(Nk,n_moviles);
velocidadRes = zeros(Nk,n_moviles);
for t=1:Nk

  for i =1:n_moviles-1     % i es el rengon de la matriz de fuerzas
     for j = i+1:n_moviles % j es la columna
   
        dr = r(:,j) - r(:,i);
    
        dr2 = dr' * dr;
    
        u = dr / sqrt(dr2);
    
        F = G * m(i)*m(j)/dr2 * u;
        Fza(i,j,:) =  F';
        Fza(j,i,:) = -F';
     end
  end

  for i = 1:n_moviles
    FzaTot(:,i) = sum(Fza(i,:,:));
  end

    a = FzaTot ./ M;
    %Se guardan las aceleraciones
    aceleracionX(t,:)=a(1,:);
    aceleracionY(t,:)=a(2,:);
    aceleracionZ(t,:)=a(3,:);

    v = v + a * deltaT;
    velocidadX(t,:)=v(1,:);
    velocidadY(t,:)=v(2,:);
    velocidadZ(t,:)=v(3,:);

    r = r + v * deltaT;

    for i = 1:n_moviles
        pos(t+1,3*(i-1)+1:3*i) = r(:,i)';
    end                
end

for i=1:Nk
    for j=1:n_moviles
        aceleracionRes(i,j) =sqrt( (aceleracionX(i,j)^2)+(aceleracionY(i,j)^2)+(aceleracionZ(i,j)^2));
        velocidadRes(i,j) = sqrt( (velocidadX(i,j)^2)+(velocidadY(i,j)^2)+(velocidadZ(i,j)^2));
    end
end

pos2 = zeros(Nk+1,3*n_moviles);
nuevoMovilCentral = 2;
for i=1:Nk+1 
    for j=1:n_moviles
        pos2(i,1+((j-1)*3))=pos(i,1+((j-1)*3))-pos(i,1+((nuevoMovilCentral-1)*3)); %Se le resta a todos los moviles la posicion en X del movil que se desea centrar.
        pos2(i,2+((j-1)*3))=pos(i,2+((j-1)*3))-pos(i,2+((nuevoMovilCentral-1)*3)); %Se le resta a todos los moviles la posicion en Y del movil que se desea centrar.
        pos2(i,3+((j-1)*3))=pos(i,3+((j-1)*3))-pos(i,3+((nuevoMovilCentral-1)*3)); %Se le resta a todos los moviles la posicion en Z del movil que se desea centrar.
    end
end

prom = zeros(1,n_moviles);
for i = 1:Nk
    for j=1:n_moviles
        prom(1,j) = prom(1,j) + ((velocidadRes(i,j)/1000)/Nk);
    end
end

xmin = min(min(pos(:,1:3:3*n_moviles)));
xmax = max(max(pos(:,1:3:3*n_moviles)));

ymin = min(min(pos(:,2:3:3*n_moviles)));
ymax = min(min(pos(:,2:3:3*n_moviles)));

zmin = min(min(pos(:,3:3:3*n_moviles)));
zmax = min(min(pos(:,3:3:3*n_moviles)));

hold off
plot3([xmin,xmax],[ymin,ymax],[xmin,xmax],'.w')
hold on
plot3(pos(:,1),pos(:,2),pos(:,3),'oy') %,'DisplayName',sprintf('Sol'))
plot3(pos(:,4),pos(:,5),pos(:,6),'-g') %,'DisplayName',sprintf('Tierra'))
plot3(pos(:,7),pos(:,8),pos(:,9),'-b') %,'DisplayName',sprintf('Luna'))
plot3(pos(:,10),pos(:,11),pos(:,12),'-r') %,'DisplayName',sprintf('Marte'))
plot3(pos(:,13),pos(:,14),pos(:,15),'-k') %,'DisplayName',sprintf('Venus'))
plot3(pos(:,16),pos(:,17),pos(:,18),'-m') %,'DisplayName',sprintf('Mercurio'))
plot3(pos(:,19),pos(:,20),pos(:,21),'-k') %,'DisplayName',sprintf('Fobos'))
plot3(pos(:,22),pos(:,23),pos(:,24),'-b') %,'DisplayName',sprintf('Deimos'))

title('Sol - Tierra - Luna - Planetas');
xlabel('Coordenada X (mts)');
ylabel('Coordenada Y (mts)');
zlabel('Coordenada Z (mts)');
[hleg1, hobj1] = legend('','Sol','Tierra','Luna','Marte','Venus','Mercurio','Fobos','Deinos');%,'Sat GeoEst','Sat Luna','Sat l1', 'Sat l2')

hold off 
pause(10)
clf
hold on
plot(1:Nk,aceleracionRes(:,1),"-y")
plot(1:Nk,aceleracionRes(:,2),"-g")
plot(1:Nk,aceleracionRes(:,3),"-b")
plot(1:Nk,aceleracionRes(:,4),"-r")
plot(1:Nk,aceleracionRes(:,5),"-k")
plot(1:Nk,aceleracionRes(:,6),"-m")
plot(1:Nk,aceleracionRes(:,7),"-k")
plot(1:Nk,aceleracionRes(:,8),"-b")
title('Tiempo-Aceleración');
[hleg1, hobj1] = legend('Sol','Tierra','Luna','Marte','Venus','Mercurio','Fobos','Deinos');%,'Sat GeoEst','Sat Luna','Sat l1', 'Sat l2')

hold off 
pause(10)
clf
hold on
plot(1:Nk,velocidadRes(:,1),"-y")
plot(1:Nk,velocidadRes(:,2),"-g")
plot(1:Nk,velocidadRes(:,3),"-b")
plot(1:Nk,velocidadRes(:,4),"-r")
plot(1:Nk,velocidadRes(:,5),"-k")
plot(1:Nk,velocidadRes(:,6),"-m")
plot(1:Nk,velocidadRes(:,7),"-k")
plot(1:Nk,velocidadRes(:,8),"-b")
title('Tiempo-Velocidad');
[hleg1, hobj1] = legend('Sol','Tierra','Luna','Marte','Venus','Mercurio','Fobos','Deinos');%,'Sat GeoEst','Sat Luna','Sat l1', 'Sat l2')


%modelo geocénntrico

xmin2 = min(min(pos(:,1:3:3*n_moviles)));
xmax2 = max(max(pos(:,1:3:3*n_moviles)));

ymin2 = min(min(pos(:,2:3:3*n_moviles)));
ymax2 = min(min(pos(:,2:3:3*n_moviles)));

zmin2 = min(min(pos(:,3:3:3*n_moviles)));
zmax2 = min(min(pos(:,3:3:3*n_moviles)));
hold off
pause(10)
clf
hold on 
plot3([xmin2,xmax2],[ymin2,ymax2],[xmin2,xmax2],'.w')
plot3(pos2(:,1),pos2(:,2),pos2(:,3),'-y') %,'DisplayName',sprintf('Sol'))
plot3(pos2(:,4),pos2(:,5),pos2(:,6),'og') %,'DisplayName',sprintf('Tierra'))
plot3(pos2(:,7),pos2(:,8),pos2(:,9),'-b') %,'DisplayName',sprintf('Luna'))
plot3(pos2(:,10),pos2(:,11),pos2(:,12),'-r') %,'DisplayName',sprintf('Marte'))
plot3(pos2(:,13),pos2(:,14),pos2(:,15),'-k') %,'DisplayName',sprintf('Venus'))
plot3(pos2(:,16),pos2(:,17),pos2(:,18),'-m') %,'DisplayName',sprintf('Mercurio'))
plot3(pos2(:,19),pos2(:,20),pos2(:,21),'-k') %,'DisplayName',sprintf('Venus'))
plot3(pos2(:,22),pos2(:,23),pos2(:,24),'-b')
title('Modelo geoéntrico');
xlabel('Coordenada X (mts)');
ylabel('Coordenada Y (mts)');
zlabel('Coordenada Z (mts)');
[hleg1, hobj1] = legend('','Sol','Tierra','Luna','Marte','Venus','Mercurio','Fobos','Deinos');%,'Sat GeoEst','Sat Luna','Sat l1', 'Sat l2')

hold off
pause(10)
clf
hold on 
plot3(pos(1:10000,10),pos(1:10000,11),pos(1:10000,12),'-r') %,'DisplayName',sprintf('Marte'))
plot3(pos(1:10000,19),pos(1:10000,20),pos(1:10000,21),'-k') %,'DisplayName',sprintf('Fobos'))
plot3(pos(1:10000,22),pos(1:10000,23),pos(1:10000,24),'-b') %,'DisplayName',sprintf('Deimos'))
title('Zoom a la orbita de Marte');
xlabel('Coordenada X (mts)');
ylabel('Coordenada Y (mts)');
zlabel('Coordenada Z (mts)');
[hleg1, hobj1] = legend('Marte','Fobos','Deinos');%,'Sat GeoEst','Sat Luna','Sat l1', 'Sat l2')
