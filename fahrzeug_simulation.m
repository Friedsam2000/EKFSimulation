clear
clc
keypress = 'x';

%% 
%Car Settings
global dt m Iz l lR lF CFR CFL CRR CRL w Rreifen Clong IRad Clat saves keyboardControl gLatModelFromAy
m = 300; %[kg] Masse
Iz = 81.25; %[kg*m^2] Trägheitsmoment um z-Achse
lF = 0.8;
lR = 0.8;
l = lR + lF;
Clat = 25000; %[N/rad]
CFR = Clat; %[N/rad] cornering stiffness of front right tire
CFL = Clat;  %[N/rad] cornering stiffness of front left tire
CRR = Clat; %[N/rad] cornering stiffness of rear right tire
CRL = Clat; %[N/rad] cornering stiffness of rear left tire
w = 1.18; %[m] Spurweite
Rreifen = 0.2;%[m] Reifenradius
Clong = 150;  %Longitudinal Force per Slip ratio [N/%slip];
IRad = 0.5; %[kg*m^2] Trägheitsmoment Rad
gLatModelFromAy = 0;

%Time settings
dt = 0.001;
maxTime = 200; %[s]
t = (0:dt:maxTime)';
N = length(t);


%Initialisation of saved Arrays
saves.sFL = zeros(1,maxTime/dt);
saves.sFR = zeros(1,maxTime/dt);
saves.sRL = zeros(1,maxTime/dt);
saves.sRR = zeros(1,maxTime/dt);
saves.delta = zeros(1,maxTime/dt);
saves.MFL = zeros(1,maxTime/dt);
saves.MFR = zeros(1,maxTime/dt);
saves.MRL = zeros(1,maxTime/dt);
saves.MRR = zeros(1,maxTime/dt);
saves.alphaFL = zeros(1,maxTime/dt);
saves.alphaFR = zeros(1,maxTime/dt);
saves.alphaRL = zeros(1,maxTime/dt);
saves.alphaRR = zeros(1,maxTime/dt);
saves.dX = zeros(1,maxTime/dt);
saves.dY = zeros(1,maxTime/dt);
saves.dphi = zeros(1,maxTime/dt);
saves.dvx = zeros(1,maxTime/dt);
saves.dvy = zeros(1,maxTime/dt);
saves.domegaFL = zeros(1,maxTime/dt);
saves.domegaFR = zeros(1,maxTime/dt);
saves.domegaRL = zeros(1,maxTime/dt);
saves.domegaRR = zeros(1,maxTime/dt);
saves.delta = zeros(1,maxTime/dt);
saves.delta_right = zeros(1,maxTime/dt);
saves.delta_left = zeros(1,maxTime/dt);
saves.ax = zeros(1,maxTime/dt);
saves.ay = zeros(1,maxTime/dt);
saves.ayFromgLat = zeros(1,maxTime/dt);
X   = zeros(1,maxTime/dt);
Y   = zeros(1,maxTime/dt);
phi = zeros(1,maxTime/dt);
vx  = zeros(1,maxTime/dt);
vy  = zeros(1,maxTime/dt);
r   = zeros(1,maxTime/dt);
omegaFL = zeros(1,maxTime/dt);
omegaFR = zeros(1,maxTime/dt);
omegaRL = zeros(1,maxTime/dt);
omegaRR = zeros(1,maxTime/dt);
vxKistler = zeros(1,maxTime/dt);
vyKistler = zeros(1,maxTime/dt);
gLat = zeros(1,maxTime/dt);
gLong = zeros(1,maxTime/dt);      
xFR = zeros(1,maxTime/dt);  
yFR = zeros(1,maxTime/dt);  
xFL  = zeros(1,maxTime/dt);  
yFL  = zeros(1,maxTime/dt);  
xRR  = zeros(1,maxTime/dt);  
yRR  = zeros(1,maxTime/dt);  
xRL  = zeros(1,maxTime/dt);  
yRL = zeros(1,maxTime/dt);  
xFL_F  = zeros(1,maxTime/dt);  
yFL_F  = zeros(1,maxTime/dt);  
xFL_R  = zeros(1,maxTime/dt);  
yFL_R  = zeros(1,maxTime/dt);  
xFR_F  = zeros(1,maxTime/dt);  
yFR_F = zeros(1,maxTime/dt);  
xFR_R  = zeros(1,maxTime/dt);  
yFR_R  = zeros(1,maxTime/dt);  
xRL_F = zeros(1,maxTime/dt);  
yRL_F  = zeros(1,maxTime/dt);  
xRL_R = zeros(1,maxTime/dt);  
yRL_R  = zeros(1,maxTime/dt);  
xRR_F = zeros(1,maxTime/dt);  
yRR_F  = zeros(1,maxTime/dt);  
xRR_R  = zeros(1,maxTime/dt);  
yRR_R  = zeros(1,maxTime/dt);  

%Initital States
X(1)   = 0;
Y(1)   = 0;
phi(1) = 0;
vx(1)  = 1; 
vy(1)  = 0;
r(1)   = 0;
omegaFL(1) = vx(1)/Rreifen;
omegaFR(1) = vx(1)/Rreifen;
omegaRL(1) = vx(1)/Rreifen;
omegaRR(1) = vx(1)/Rreifen;
S(:,1) = [X(1);Y(1);phi(1);vx(1);vy(1);r(1) ;omegaFL(1);omegaFR(1);omegaRL(1);omegaRR(1)];

%Input settings
steeringFaktor = 0.1;
momentFaktor = 0.1;
selfaligning = 0;
selfZeroThrottle = 0;

%Keyboard Control on/off
keyboardControl = 1;

%Initialise Controls
delta = repelem(0,N);
MFL = repelem(0,N);
MFR = repelem(0,N);
MRL = repelem(0,N);
MRR = repelem(0,N);

%Speed Control on/off
global speedController controllerFaktor vSpeedCtrl
speedController = 	0;
controllerFaktor = 	20;
vSpeedCtrl = 31/3.6;

%GSS Position in Car Coordinates
xKistler = 1;
yKistler = 1;

%Simulation settings
doSimulation = 1;
skip = 16*8; %only draw ever x frames
zoom = 7; %zoom factor for simulation window (5 for car view, 15 for track view)


%% Simulation

for k = 1:N-1

    %Automatic driving programs (comment in to drive automatically)
    
%     accel
    if k > 1000
        if X(k-1) < 75
            speedController = 	1;
            controllerFaktor = 	20;
            vSpeedCtrl = 120/3.6;
            delta(k) = 0;
            delta(k) = 0;
        elseif vx(k-1) > 0
            speedController = 	1;
            controllerFaktor = 	10;
            vSpeedCtrl = 0;
            delta(k) = 0;
        end
        if X(k-1) > 90 && vx(k-1) < 2
            keypress = 'c';
        end
    end
    
% %     skidpad
%         if k>1+1070 && S(4,k) < 9.5940
%                 MFL(k) = 70;
%                 MFR(k) = 70;
%                 MRL(k) = 70;
%                 MRR(k) = 70;
%                 delta(k) = 0;
%         end
%         if k>1+1070 && S(4,k) >= 9.5940
%                 MFL(k) = 0;
%                 MFR(k) = 0;
%                 MRL(k) = 0;
%                 MRR(k) = 0;
%                 delta(k) = 0;
%         end
%         if k>2300+1070 && delta(k) < 0.2
%             speedController = 	1;
%             vSpeedCtrl = 9.5940;
%             delta(k) = delta(k-1) + (0.2/50);
%         end
%         if k>2300+1070 && delta(k) >= 0.2
%             delta(k) = 0.2;
%         end
%         
%         if k>13472+1120 && delta(k) > -0.2
%             delta(k) = delta(k-1) - (0.2/50);
%         end
%         if k>13472+1120 && delta(k) <= -0.2
%             delta(k) = -0.2;
%         end
%         if k>24000+1740 && delta(k) < -0.0015
%             delta(k) = delta(k-1) + (0.15/400);
%         end
%         if k>24000+1740 && delta(k) >= -0.0015
%             delta(k) = 0;
%         end
%         
%         if k>(25000+2000) && vx(k-1) > 2
%             speedController = 	0;
%             MFL(k) = -70;
%             MFR(k) = -70;
%             MRL(k) = -70;
%             MRR(k) = -70;
%             delta(k) = 0;
%         end
%       
%         if k>(25000+2000) && vx(k-1)<2
%             keypress = 'c';
%         end
    
    %% Model

    %Vehicle Model
    S(:,k+1) = dynamicModel([S(:,k);delta(k);MFL(k);MFR(k);MRL(k);MRR(k)],k);

    %Wrap Phi to 0 - 2pi
    S(3,k) = mod(S(3,k),2*pi);
    
    %% Calculations
    %Expose States from S
    X(k)   = S(1,k);
    Y(k)   = S(2,k);
    phi(k) = S(3,k);
    vx(k)  = S(4,k);
    vy(k)  = S(5,k);
    r(k)   = S(6,k);
    omegaFL(k) = S(7,k);
    omegaFR(k) = S(8,k);
    omegaRL(k) = S(9,k);
    omegaRR(k) = S(10,k);
    
    % Speed at Kistler
    vKistler = getLocalSpeed(xKistler,yKistler,vx(k),vy(k),r(k));
    vxKistler(k) = vKistler(1);
    vyKistler(k) = vKistler(2);
    

    global gLatModel gLongModel 
    if k > 1
        % g lateral
        gLat(k) = gLatModel;

        % g longitudinal
        gLong(k) = gLongModel;
    end
    
    %% Simulation
    if doSimulation == 1 && mod(k,skip) == 0
        % animating vehicle as a function of time
        
        xFR(k) = X(k) + lF*cos(phi(k)) + (w/2)*sin(phi(k));
        yFR(k) = Y(k) + lF*sin(phi(k)) - (w/2)*cos(phi(k));
        xFL(k) = X(k) + lF*cos(phi(k)) - (w/2)*sin(phi(k));
        yFL(k) = Y(k) + lF*sin(phi(k)) + (w/2)*cos(phi(k));

        xRR(k) = X(k) - lR*cos(phi(k)) + (w/2)*sin(phi(k));
        yRR(k) = Y(k) - lR*sin(phi(k)) - (w/2)*cos(phi(k));
        xRL(k) = X(k) - lR*cos(phi(k)) - (w/2)*sin(phi(k));
        yRL(k) = Y(k) - lR*sin(phi(k)) + (w/2)*cos(phi(k));

        
        Rt = Rreifen*2; %geplotteter Reifenradius
        xFL_F(k) = xFL(k) + Rt*cos(phi(k)+saves.delta_left(k));% x pos front of front left tire
        yFL_F(k) = yFL(k) + Rt*sin(phi(k)+saves.delta_left(k));% y pos front of front left tire
        xFL_R(k) = xFL(k) - Rt*cos(phi(k)+saves.delta_left(k));% x pos rear of front left tire
        yFL_R(k) = yFL(k) - Rt*sin(phi(k)+saves.delta_left(k));% y pos rear of front left tire
        
        
        xFR_F(k) = xFR(k) + Rt*cos(phi(k)+saves.delta_right(k));% x pos front of front right tire
        yFR_F(k) = yFR(k) + Rt*sin(phi(k)+saves.delta_right(k));% y pos front of front right tire
        xFR_R(k) = xFR(k) - Rt*cos(phi(k)+saves.delta_right(k));% x pos rear of front right tire
        yFR_R(k) = yFR(k) - Rt*sin(phi(k)+saves.delta_right(k));% y pos rear of front right tire
        
        xRL_F(k) = xRL(k) + Rt*cos(phi(k));% x pos front of rear left tire
        yRL_F(k) = yRL(k) + Rt*sin(phi(k));% y pos front of rear left tire
        xRL_R(k) = xRL(k) - Rt*cos(phi(k));% x pos rear of rear left tire
        yRL_R(k) = yRL(k) - Rt*sin(phi(k));% y pos rear of rear left tire

        xRR_F(k) = xRR(k) + Rt*cos(phi(k));% x pos front of rear right tire
        yRR_F(k) = yRR(k) + Rt*sin(phi(k));% y pos front of rear right tire
        xRR_R(k) = xRR(k) - Rt*cos(phi(k));% x pos rear of rear right tire
        yRR_R(k) = yRR(k) - Rt*sin(phi(k));% y pos rear of rear right tire
        

        h_fig = figure(101);
        %Path
        plot(X(1:k),Y(1:k),'c-','linewi',2),grid on; hold on;
        %Front left tire
        plot([xFL_R(k) xFL_F(k)], [yFL_R(k) yFL_F(k)],'r-','linewi',8);
        %Front right tire
        plot([xFR_R(k) xFR_F(k)], [yFR_R(k) yFR_F(k)],'r-','linewi',8);
        %Rear left tire
        plot([xRL_R(k) xRL_F(k)], [yRL_R(k) yRL_F(k)],'k-','linewi',8);
        %Rear right tire
        plot([xRR_R(k) xRR_F(k)], [yRR_R(k) yRR_F(k)],'k-','linewi',8);
        %Rear left tire
        plot(xRL(k),yRL(k),'ko','markersize',6,'linewi',4);
        %Rear right tire
        plot(xRR(k),yRR(k),'ko','markersize',6,'linewi',4);
        %Middle line
        plot([(xRR(k)+xRL(k))/2 (xFR(k)+xFL(k))/2], [(yRR(k)+yRL(k))/2 (yFR(k)+yFL(k))/2], 'k-','linewi',2);
        %Rear Axle
        plot([xRR(k) xRL(k)], [yRR(k) yRL(k)], 'k-','linewi',4);
        %Front Axle
        plot([xFR(k) xFL(k)], [yFR(k) yFL(k)], 'k-','linewi',4);                              
        %COG
        plot(X(k),Y(k),'ko','markersize',6,'linewi',4);
        
        
%         %COG Speed Vector line
%         hlen = l*1.5;
%         xCentre = X(k);
%         yCentre = Y(k);
%         Orientation = -phi(k)-atan(vy(k)/vx(k));
%         cosOrient = cos(Orientation);
%         sinOrient = sin(Orientation);
%         xcoords = xCentre + hlen * [cosOrient -cosOrient];
%         ycoords = yCentre + hlen * [-sinOrient sinOrient];
%         line(xcoords, ycoords,'Color','green','LineStyle','-','LineWidth',1.5);

        %Keypress detection
        set(h_fig,'KeyPressFcn',@keyPressFunction);

        hold off;
        axis equal
        axis([X(k)-lF*zoom X(k)+lF*zoom Y(k)-lF*zoom Y(k)+lF*zoom])

        %Text on Plot
        printStringSteer = ['Lenkwinkel: ', num2str(round(delta(k)*180/pi,2)), ' [deg]'];
        text(X(k)+zoom/4,Y(k)+zoom/1.5-2,printStringSteer)
        printStringMoment = ['Radmoment: ', num2str(round(mean((saves.MFL(k)+saves.MFR(k)+saves.MRL(k)+saves.MRR(k)))/4,1)), ' [N/m]'];
        text(X(k)+zoom/4,Y(k)+zoom/1.5-3,printStringMoment)
        printStringSpeed = ['Geschw.: ', num2str(round(vx(k)*3.6,1)), ' [km/H]'];
        text(X(k)+zoom/4,Y(k)+zoom/1.5-4,printStringSpeed)
        printStringTime = ['Zeit: ', num2str(round(t(k),3)), ' [s]'];
        text(X(k)+zoom/4,Y(k)+zoom/1.5-5,printStringTime)
        printStringgLat = ['gLat: ', num2str(round(gLat(k),3)), ' [g]'];
        text(X(k)+zoom/4,Y(k)+zoom/1.5-6,printStringgLat)
        printStringgLong = ['gLong: ', num2str(round(gLong(k),3)), ' [g]'];
        text(X(k)+zoom/4,Y(k)+zoom/1.5-7,printStringgLong)

        drawnow
        
    end
    
    %Abort Simulation
    if keypress == 'c'
        break;
    end
    
    if keyboardControl
        %Nothing
        if ~(keypress == 'r' || keypress == 'l')
            if selfaligning == 0
                delta(k+1) = delta(k);
            else
                if delta(k) > 0
                    delta(k+1) = delta(k) - 0.01 * steeringFaktor/skip;
                end
                if delta(k) < 0
                    delta(k+1) = delta(k) + 0.01 * steeringFaktor/skip;
                end
                if delta(k) == 0
                    delta(k+1) = delta(k);
                end
            end
        end
        if ~(keypress == 'u' || keypress == 'd')
            if selfZeroThrottle == 0
                MFL(k+1) = MFL(k);
                MFR(k+1) = MFR(k);
                MRL(k+1) = MRL(k);
                MRR(k+1) = MRR(k);
            else
                if MFL(k) > 0
                    MFL(k+1) = MFL(k) - 50 * momentFaktor/skip;
                    MFR(k+1) = MFR(k) - 50 * momentFaktor/skip;
                    MRL(k+1) = MRL(k) - 50 * momentFaktor/skip;
                    MRR(k+1) = MRR(k) - 50 * momentFaktor/skip;
                end
                if MFL(k) < 0
                    MFL(k+1) = MFL(k) + 50 * momentFaktor/skip;
                    MFR(k+1) = MFR(k) + 50 * momentFaktor/skip;
                    MRL(k+1) = MRL(k) + 50 * momentFaktor/skip;
                    MRR(k+1) = MRR(k) + 50 * momentFaktor/skip;
                end
                if MFL(k) == 0
                    MFL(k+1) = MFL(k);
                    MFR(k+1) = MFR(k);
                    MRL(k+1) = MRL(k);
                    MRR(k+1) = MRR(k);
                end
            end
        end

        %Steering
        if keypress == 'r' || keypress == 'l'
            if keypress == 'r'
                delta(k+1) = delta(k) - 0.05 * steeringFaktor;
                keypress = 'x';
            end
            if keypress == 'l'
                delta(k+1) = delta(k) + 0.05 * steeringFaktor;
                keypress = 'x';
            end
        end
        %Throttle
        if keypress == 'u'
            MFL(k+1) = MFL(k) + 100 * momentFaktor;
            MFR(k+1) = MFR(k) + 100 * momentFaktor;
            MRL(k+1) = MRL(k) + 100 * momentFaktor;
            MRR(k+1) = MRR(k) + 100 * momentFaktor;
            keypress = 'x';
        end
        if keypress == 'd'
            MFL(k+1) = MFL(k) - 100 * momentFaktor;
            MFR(k+1) = MFR(k) - 100 * momentFaktor;
            MRL(k+1) = MRL(k) - 100 * momentFaktor;
            MRR(k+1) = MRR(k) - 100 * momentFaktor;
            keypress = 'x';
        end   
    end
end

%% Plotting

% Plot Path
plot(X(1:k),Y(1:k),'LineWidth',1.5);
axis equal
title('Linien Plot des gefahren Pfades');
xlabel('X [m]');
ylabel('Y [m]');
grid on
set(gca,'FontSize',22)

% Plot speed
% plot(t(1:k),vx(1:k)*3.6,'LineWidth',2);
% title('Linien Plot der Geschwindigkeit');
% xlabel('t [s]');
% ylabel('vx [km/h]');
% grid on
% set(gca,'FontSize',22)

% Plot slip
%  plot(t(1:k),saves.sFL(1:k),'LineWidth',2);
% title('Linien Plot des Längsschlupf');
% xlabel('t [s]');
% ylabel('s [%]');
% grid on
% set(gca,'FontSize',22)

% Plot Moment
%  plot(t(1:k),saves.MFL(1:k),'LineWidth',2);
% title('Linien Plot des Moments');
% xlabel('t [s]');
% ylabel('MFL [NM]');
% grid on
% set(gca,'FontSize',22)

% Plot gLat
%  plot(t(1:k),gLat(1:k));
% title('Linien Plot der Querbeschleunigung','LineWidth',3);
% xlabel('t [s]');
% ylabel('Querbeschleunigung [g]');
% grid on
% set(gca,'FontSize',22)


%% Export Simulation Results
clearvars h_fig

%Cut Time Vector
N = length(vx);
t = t(1:N);

save('simulationResults.mat');



%% Function definitions

function keyPressFunction(src,event)
   
   
   if strcmp(event.Key,'rightarrow')
       assignin('base','keypress','r');
   end
   if strcmp(event.Key,'leftarrow')
       assignin('base','keypress','l');
   end
   if strcmp(event.Key,'uparrow')
       assignin('base','keypress','u');
   end
   if strcmp(event.Key,'downarrow')
       assignin('base','keypress','d');
   end
   if strcmp(event.Key,'c')
       disp('C Pressed: stop Simulation');
       assignin('base','keypress','c');
   end
   
   
end

function vM = getLocalSpeed(xM,yM,vx,vy,r)

        vMx = vx - r*yM;
        vMy = vy + r*xM;
        
        vM = [vMx,vMy];


end

function S_new = dynamicModel(S,k)

    X = S(1);
    Y = S(2);
    phi = S(3);
    vx = S(4);
    vy = S(5);
    r = S(6);
    omegaFL = S(7);
    omegaFR = S(8);
    omegaRL = S(9);
    omegaRR = S(10);
    delta = S(11);
    MFL = S(12);
    MFR = S(13);
    MRL = S(14);
    MRR = S(15);

    % Get Global Car Settings
    global dt m Iz l  lR lF CFR CFL CRR CRL w Rreifen Clong IRad speedController controllerFaktor gLatModel gLongModel vSpeedCtrl saves

    %Ackermann steering
    delta_inner = atan((2*l*sin(delta))/(2*l*cos(delta)-w*sin(delta)));
    delta_outer = atan((2*l*sin(delta))/(2*l*cos(delta)+w*sin(delta)));
    
    if delta_inner < 0
        delta_left = delta_outer;
        delta_right = delta_inner;
    elseif delta_inner > 0
        delta_left = delta_inner;
        delta_right = delta_outer;
    else
        delta_left = 0;
        delta_right = 0;
    end

     saves.delta(k) = delta;
     saves.delta_left(k) = delta_left;
     saves.delta_right(k) = delta_right;
    
    groundSpeedFL = getLocalSpeed(lF,w/2,vx,vy,r);
    groundSpeedFR = getLocalSpeed(lF,-w/2,vx,vy,r);
    groundSpeedRL = getLocalSpeed(-lR,w/2,vx,vy,r);
    groundSpeedRR = getLocalSpeed(-lR,-w/2,vx,vy,r);
    vxFL = groundSpeedFL(1);
    vyFL = groundSpeedFL(2);
    vxFR = groundSpeedFR(1);
    vyFR = groundSpeedFR(2);
    vxRL = groundSpeedRL(1);
    vyRL = groundSpeedRL(2);
    vxRR = groundSpeedRR(1);
    vyRR = groundSpeedRR(2);
    
    %Slip angles
    alphaFL = atan(vyFL/vxFL) - delta;
    alphaFR = atan(vyFR/vxFR) - delta;
    alphaRL = atan(vyRL/vxRL);
    alphaRR = atan(vyRR/vxRR);
    
    %Clamp Slip angle to 2 deg
    maxAlpha = 2;
    alphaFL = bound(alphaFL,-(maxAlpha*pi/180),(maxAlpha*pi/180));
    alphaFR = bound(alphaFR,-(maxAlpha*pi/180),(maxAlpha*pi/180));
    alphaRL = bound(alphaRL,-(maxAlpha*pi/180),(maxAlpha*pi/180));
    alphaRR = bound(alphaRR,-(maxAlpha*pi/180),(maxAlpha*pi/180));
    
    saves.alphaFL(k) = alphaFL;
    saves.alphaFR(k) = alphaFR;
    saves.alphaRL(k) = alphaRL;
    saves.alphaRR(k) = alphaRR;
    
    %Lateral Kraft
    FFLy = -alphaFL*CFL;
    FFRy = -alphaFR*CFR;
    FRLy = -alphaRL*CRL;
    FRRy = -alphaRR*CRR;
    
    if speedController
        if (vSpeedCtrl-vx) < 0 %abbremsen
            MFL  = controllerFaktor*(vSpeedCtrl-vx); %P Regler
            MFR  = controllerFaktor*(vSpeedCtrl-vx); %P Regler
            MRL  = controllerFaktor*(vSpeedCtrl-vx); %P Regler
            MRR  = controllerFaktor*(vSpeedCtrl-vx); %P Regler
            MFL = bound(MFL,-300,300);
            MFR = bound(MFR,-300,300);
            MRL = bound(MRL,-300,300);
            MRR = bound(MRR,-300,300);
        else
            MFL  = controllerFaktor*(vSpeedCtrl-vx); %P Regler
            MFR  = controllerFaktor*(vSpeedCtrl-vx); %P Regler
            MRL  = controllerFaktor*(vSpeedCtrl-vx); %P Regler
            MRR  = controllerFaktor*(vSpeedCtrl-vx); %P Regler
            MFL = bound(MFL,-300,300);
            MFR = bound(MFR,-300,300);
            MRL = bound(MRL,-300,300);
            MRR = bound(MRR,-300,300);
        end
            
    end
    
    
    %Slip Ratios
    sFL = 100*((omegaFL*Rreifen)/(vxFL*cos(delta))-1);
    sFR = 100*((omegaFR*Rreifen)/(vxFR*cos(delta))-1);
    sRL = 100*((omegaRL*Rreifen)/vxRL-1);
    sRR = 100*((omegaRR*Rreifen)/vxRR-1);

    %Clamp Slip to 8.47%
    maxSlip = 8.47;
    sFL = bound(sFL,-maxSlip,maxSlip);
    sFR = bound(sFR,-maxSlip,maxSlip);
    sRL = bound(sRL,-maxSlip,maxSlip);
    sRR = bound(sRR,-maxSlip,maxSlip);
    

    saves.sFL(k) = sFL;
    saves.sFR(k) = sFR;
    saves.sRL(k) = sRL;
    saves.sRR(k) = sRR;
    
    saves.MFL(k) = MFL;
    saves.MFR(k) = MFR;
    saves.MRL(k) = MRL;
    saves.MRR(k) = MRR;
    
    %Longitudinal Force from Slip
    FFLx = Clong*sFL;
    FFRx = Clong*sFR;
    FRLx = Clong*sRL;
    FRRx = Clong*sRR;


    %4W Model
    dX   = vx*cos(phi) - vy*sin(phi);
    dY   = vx*sin(phi) + vy*cos(phi);
    dphi = r;
    dvx  = (1/m)* (FRLx+FRRx+FFLx*cos(delta_left)+FFRx*cos(delta_right)-FFLy*sin(delta_left)-FFRy*sin(delta_right)+m*vy*r);
    dvy  = (1/m)* (FRLy+FRRy+FFLy*cos(delta_left)+FFRy*cos(delta_right)+FFLx*sin(delta_left)+FFRx*sin(delta_right)-m*vx*r);
    dr   = (1/Iz)*(-FRLy*lR-FRLx*(w/2)-FRRy*lR+FRRx*(w/2)+FFLy*sin(delta_left)*(w/2)+FFLy*cos(delta_left)*lF-FFLx*cos(delta_left)*(w/2) ...
              +FFLx*sin(delta_left)*lF+FFRx*cos(delta_right)*(w/2)+FFRx*sin(delta_right)*lF+FFRy*cos(delta_right)*lF-FFRy*sin(delta_right)*(w/2));
    domegaFL = (MFL - FFLx*Rreifen)/IRad;
    domegaFR = (MFR - FFRx*Rreifen)/IRad;
    domegaRL = (MRL - FRLx*Rreifen)/IRad;
    domegaRR = (MRR - FRRx*Rreifen)/IRad;
    
    saves.dX(k) = dX;
    saves.dY(k) = dY;
    saves.dphi(k) = dphi;
    saves.dvx(k) = dvx;
    saves.dvy(k) = dvy;
    saves.domegaFL(k) = domegaFL;
    saves.domegaFR(k) = domegaFR;
    saves.domegaRL(k) = domegaRL;
    saves.domegaRR(k) = domegaRR;

    dS = [dX,dY,dphi,dvx,dvy,dr,domegaFL,domegaFR,domegaRL,domegaRR]';
    S_old = S(1:10);
    %Euler Step
    S_new = (S_old + dS *dt)';
    
   gLongModel = (1/9.81)*(1/m)*(FRLx+FRRx+FFLx*cos(delta)+FFRx*cos(delta)-FFLy*sin(delta)-FFRy*sin(delta));
   gLatModel = (1/9.81)*(1/m)*(FRLy+FRRy+FFLy*cos(delta)+FFRy*cos(delta)+FFLx*sin(delta)+FFRx*sin(delta));

   saves.ax(k) = gLongModel*9.81;
   saves.ay(k) = gLatModel*9.81;
    
end


function y = bound(x,bl,bu)
  % return bounded value clipped between bl and bu
  y=min(max(x,bl),bu);
end
