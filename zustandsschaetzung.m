clear all
close all


%% Import SensorSimulation
fullMatFileName = 'C:\Users\samue\Documents\Git\VehicleModel_SE_Simulation\elsd\ros_ws\src\2_estimation\elsd_vehiclemodel_stateestimation\EKF_Samuel\sensorResults.mat';
if ~exist(fullMatFileName, 'file')
  message = sprintf('Run FourWheelModel.m first to generate Results', fullMatFileName);
  uiwait(warndlg(message));
else
  sim = load(fullMatFileName);
end
%Time Definition
dt = sim.dt;
t = sim.t;
N = sim.k;

%Initialisation
f_ins_position = 0;
f_ins_heading = 0;
f_ins_velocity = 0;
f_imu = 0;
f_odometry = 0;
f_gss = 0;


%% Control Segment

% %Errors in Parameters (only affects odometry sensor)
sim.Rreifen = sim.Rreifen * 1.02; 
sim.Clong = sim.Clong * 1.5;
sim.IRad = sim.IRad  * 1.5;

%(De-)Activation of Sensors [Hz]
f_ins_position = 20; 
f_ins_heading = 20; 
f_ins_velocity = 20; 
f_imu = 100; 
f_odometry = 60;
f_gss = 100;

% P0
initialUncertainty = [0,0,0,1,0,0,0,0,4,4,4,4];

%S0
S(1,1) = 0;
S(2,1) = 0;
S(3,1) = 0;
S(4,1) = 0;
S(5,1) = 0;
S(6,1) = 0;
S(7,1) = 0;
S(8,1) = 0;
S(9,1) = 0;
S(10,1) = 0;
S(11,1) = 0;
S(12,1) = 0;


%% Initialisiations

%Initiale Kovariant Matrix
P(:,:,1) = diag(initialUncertainty);

ins_position_active_every_x_s = 1/f_ins_position;
ins_heading_active_every_x_s = 1/f_ins_heading;
ins_velocity_active_every_x_s = 1/f_ins_velocity;
imu_active_every_x_s = 1/f_imu;
odometry_active_every_x_s = 1/f_odometry; 
gss_active_every_x_s = 1/f_gss;

ins_position_inactive_time = 0;
ins_heading_inactive_time = 0;
ins_velocity_inactive_time = 0;
imu_inactive_time = 0;
odometry_inactive_time = 0;
gss_inactive_time = 0;


for i = 1:N
    %ins_position
    if ins_position_inactive_time >= ins_position_active_every_x_s
        ins_position_active(i) = 1;
        ins_position_inactive_time = 0;
    else
        ins_position_active(i) = 0;
    end
    %ins_heading
    if ins_heading_inactive_time >= ins_heading_active_every_x_s
        ins_heading_active(i) = 1;
        ins_heading_inactive_time = 0;
    else
        ins_heading_active(i) = 0;
    end
    %ins_velocity
    if ins_velocity_inactive_time >= ins_velocity_active_every_x_s
        ins_velocity_active(i) = 1;
        ins_velocity_inactive_time = 0;
    else
        ins_velocity_active(i) = 0;
    end
    %imu
    if imu_inactive_time >= imu_active_every_x_s
        imu_active(i) = 1;
        imu_inactive_time = 0;
    else
        imu_active(i) = 0;
    end
    %odometry
    if odometry_inactive_time >= odometry_active_every_x_s
        odometry_active(i) = 1;
        odometry_inactive_time = 0;
    else
        odometry_active(i) = 0;
    end
    %gss
    if gss_inactive_time >= gss_active_every_x_s
        gss_active(i) = 1;
        gss_inactive_time = 0;
    else
        gss_active(i) = 0;
    end
    
    ins_position_inactive_time = ins_position_inactive_time + dt;
    ins_heading_inactive_time = ins_heading_inactive_time + dt;
    ins_velocity_inactive_time = ins_velocity_inactive_time + dt;
    imu_inactive_time = imu_inactive_time + dt;
    odometry_inactive_time = odometry_inactive_time + dt;
    gss_inactive_time = gss_inactive_time + dt;
    
end
%% Simulate GPS Signal Loss

% total_timesteps = N+1;
% 
ins_position_active(1:110000)=0;
ins_heading_active(1:110000)=0;
ins_velocity_active(1:110000)=0;




%% Other Definitions

k = 2;
i_last_iteration = 1;
i_last_odometry_iteration = 1;
tic

%% Iteration über Zeitschritte der Simulation
for i = 2 : N
    
    %% Kalman Predict - Do when new Observation arrives
    if ins_position_active(i)||ins_heading_active(i)||ins_velocity_active(i)||imu_active(i)||odometry_active(i)||gss_active(i)

        %Time Calculations
        dtKalman = t(i) - t(i_last_iteration);
        tStates(k) = t(i);
        
        %Get Wheel Moments and Steering angle of the last odometry timestep
        MFL = sim.saves.MFL(i_last_odometry_iteration);
        MFR = sim.saves.MFR(i_last_odometry_iteration);
        MRL = sim.saves.MRL(i_last_odometry_iteration);
        MRR = sim.saves.MRR(i_last_odometry_iteration);
        delta = sim.deltaMeas(i_last_odometry_iteration);
        
        % Q
        modelMismatchVector = [0.00001,0.00001,0.00001,0.1,0.01,0.01,1,1,100000,100000,100000,100000];
        Q = diag(dtKalman*modelMismatchVector); 
        
        vx_use_wheelspeed = 1; %use wheelspeed above given speed value
        %Predicition
        if S(4,k-1)>vx_use_wheelspeed
            [S(:,k),P(:,:,k)] = predictDynamic(S(:,k-1),Q,P(:,:,k-1),delta,MFL,MFR,MRL,MRR,sim.Clong,dtKalman,sim.Rreifen,sim.w,sim.IRad);
        end
        if S(4,k-1)<=vx_use_wheelspeed 
            [S(:,k),P(:,:,k)] = predictDynamicWithoutWheelspeed(S(:,k-1),P(:,:,k-1),dtKalman,Q); %predictDynamic not defined for slow speeds
        end

        
    %% Kalman Correct durchführen wenn der entsprechende Messwert vorliegt  

        if ins_position_active(i) == 1
           [S(:,k),P(:,:,k)] =  make_ins_position_measurement(S(:,k),P(:,:,k),sim.XGNSSMeas(i),sim.YGNSSMeas(i));
        end
        if ins_heading_active(i) == 1
            [S(:,k),P(:,:,k)] =  make_ins_heading_measurement(S(:,k),P(:,:,k),sim.phiGNSSMeas(i));
        end
        if ins_velocity_active(i) == 1
            [S(:,k),P(:,:,k)] =  make_ins_velocity_vehicle_measurement(S(:,k),P(:,:,k),sim.vxGNSSMeas(i),sim.vyGNSSMeas(i));
        end
        if imu_active(i) == 1
            [S(:,k),P(:,:,k)] =  make_imu_measurement(S(:,k),P(:,:,k),sim.rIMUMeas(i),sim.axIMUMeas(i),sim.ayIMUMeas(i));
        end
        if odometry_active(i) == 1
            [S(:,k),P(:,:,k)] =  make_odometry_measurement(S(:,k),P(:,:,k),sim.omegaFLMeas(i),sim.omegaFRMeas(i),sim.omegaRLMeas(i),sim.omegaRRMeas(i));
            i_last_odometry_iteration = i;
        end
        if gss_active(i) == 1
            [S(:,k),P(:,:,k)] =  make_gss_measurement(S(:,k),P(:,:,k),sim.vxGSSMeas(i),sim.vyGSSMeas(i),sim.xKistler,sim.yKistler);
        end
      
         k = k + 1;
         i_last_iteration = i;
         
    end

end

toc

%% Plotting

%calc stdv
 stdvx = sqrt(P(4,4,:));
 stdvy = sqrt(P(5,5,:));

% % Path
% plot(S(1,:),S(2,:));
% hold on
% xlabel('X [m]');
% ylabel('Y [m]');
% grid on
% plot(sim.Xtrue,sim.Ytrue);
% axis equal
% set(gca,'FontSize',22)
% legend('PathKalman','PathTrue');
% 
% % X
% plot(tStates,S(1,:),'LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980]);
% hold on
% stdX = sqrt(P(1,1,:));
% plot(tStates,S(1,:)+stdX(:)','--','LineWidth',1.5,'Color',[0.9290, 0.6940, 0.1250]);
% plot(tStates,S(1,:)-stdX(:)','--','LineWidth',1.5,'Color',[0.4940 0.1840 0.5560]);
% plot(t(1:N),sim.Xtrue,'LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
% xlabel('t [s]');
% ylabel('X Pos. [m]');
% grid on
% set(gca,'FontSize',22)
% legend('X geschätzt','  +\sigma','  -\sigma','X wahr');
% 
% Y
% plot(tStates,S(2,:),'LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980]);
% hold on
% stdY = sqrt(P(2,2,:));
% plot(tStates,S(2,:)+stdY(:)','--','LineWidth',1.5,'Color',[0.9290, 0.6940, 0.1250]);
% plot(tStates,S(2,:)-stdY(:)','--','LineWidth',1.5,'Color',[0.4940 0.1840 0.5560]);
% plot(t(1:N),sim.Ytrue,'LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
% xlabel('t [s]');
% ylabel('Y Pos. [m]');
% grid on
% set(gca,'FontSize',22)
% legend('YKalman','+StdY.','-StdY.','YTrue');
% 
% % Heading
% plot(tStates,S(3,:),'LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980]);
% hold on
% stdphi = sqrt(P(3,3,:));
% plot(tStates,S(3,:)+stdphi(:)','--','LineWidth',1.5,'Color',[0.9290, 0.6940, 0.1250]);
% plot(tStates,S(3,:)-stdphi(:)','--','LineWidth',1.5,'Color',[0.4940 0.1840 0.5560]);
% plot(t(1:N),sim.phiTrue,'LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
% xlabel('t [s]');
% ylabel('phi [rad]');
% grid on
% set(gca,'FontSize',22)
% legend('phiKalman','+StdPhi.','-StdPhi.','phiTrue');

% vx
plot(tStates,S(4,:),'LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980]);
hold on
plot(tStates,S(4,:)+stdvx(:)','--','LineWidth',1.5,'Color',[0.9290, 0.6940, 0.1250]);
plot(tStates,S(4,:)-stdvx(:)','--','LineWidth',1.5,'Color',[0.4940 0.1840 0.5560]);
plot(t(1:N),sim.vxTrue,'LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
xlabel('t [s]');
ylabel('vx [m/s]');
grid on
set(gca,'FontSize',22)
legend('vx geschätzt','  +\sigma','  -\sigma','vx wahr');

% % vy
% plot(tStates,S(5,:),'LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980]);
% hold on
% plot(tStates,S(5,:)+stdvy(:)','--','LineWidth',1.5,'Color',[0.9290, 0.6940, 0.1250]);
% plot(tStates,S(5,:)-stdvy(:)','--','LineWidth',1.5,'Color',[0.4940 0.1840 0.5560]);
% plot(t(1:N),sim.vyTrue,'LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
% xlabel('t [s]');
% ylabel('vy [m/s]');
% grid on
% set(gca,'FontSize',22)
% legend('vy geschätzt','  +\sigma','  -\sigma','vy wahr');
% % 
% % r
% plot(tStates,S(6,:),'LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980]);
% hold on
% stdr = sqrt(P(6,6,:));
% plot(tStates,S(6,:)+stdr(:)','--','LineWidth',1.5,'Color',[0.9290, 0.6940, 0.1250]);
% plot(tStates,S(6,:)-stdr(:)','--','LineWidth',1.5,'Color',[0.4940 0.1840 0.5560]);
% plot(t(1:N),sim.rTrue,'LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
% xlabel('t [s]');
% ylabel('r [rad/s]');
% grid on
% set(gca,'FontSize',22)
% legend('rKalman','+Stdr.','-Stdr.','rTrue');

%Calculate Max and Mean Errors
for i = 1000 : N
    indexState = find(tStates==t(i));
    if indexState
        vxError(indexState) = abs(sim.vxTrue(i) - S(4,indexState));
        vyError(indexState) = abs(sim.vyTrue(i) - S(5,indexState));
        pathError(indexState) = sqrt((sim.Xtrue(i) - S(1,indexState))^2+(sim.Ytrue(i) - S(2,indexState))^2);
        gierratenError(indexState) = abs(sim.rTrue(i) - S(6,indexState));
        axError(indexState) = abs(sim.axTrue(i)- S(7,indexState));
        ayError(indexState) = abs(sim.ayTrue(i)- S(8,indexState));
    end
end
disp(horzcat('Max Pos Error = ', num2str(max(pathError)), ' m'));
disp(horzcat('Mean Pos Error = ', num2str(mean(pathError)), ' m'));
disp(horzcat('Max vx Error = ', num2str(max(vxError)), ' m/s'));
disp(horzcat('Mean vx Error = ', num2str(mean(vxError)), ' m/s'));
disp(horzcat('Mean vx Stdv = ', num2str(mean(stdvx)), ' m/s'));
disp(horzcat('Max vy Error = ', num2str(max(vyError)), ' m/s'));
disp(horzcat('Mean vy Error = ', num2str(mean(vyError)), ' m/s'));
disp(horzcat('Mean vy Stdv = ', num2str(mean(stdvy)), ' m/s'));


function [S,P] = predictDynamic(S,Q,P,delta,MFL,MFR,MRL,MRR,Clong,dtKalman,tireRadius,w,IRad)

        %% Prediction
        
        % Write state as letters to easily understand the calculation in the predict step
        X = S(1);
        Y = S(2);
        phi =S(3);
        vx =S(4);
        vy = S(5);
        r = S(6);
        ax =S(7);
        ay =S(8);
        omegaFL =S(9);
        omegaFR =S(10);
        omegaRL =S(11);
        omegaRR =S(12);
        
        
            S =   [         X+dtKalman*(vx*cos(phi) - vy*sin(phi));
                            Y+dtKalman*(vx*sin(phi) + vy*cos(phi));
                            phi+dtKalman*(r);
                            vx+dtKalman*(ax+r*vy);
                            vy+dtKalman*(ay-r*vx);
                            r;
                            ax;
                            ay;
                            omegaFL+dtKalman*((MFL - Clong*(100*((omegaFL*tireRadius)/((vx - r*(w/2))*cos(delta))-1))*tireRadius)/IRad);
                            omegaFR+dtKalman*((MFR - Clong*(100*((omegaFR*tireRadius)/((vx - r*(-w/2))*cos(delta))-1))*tireRadius)/IRad);
                            omegaRL+dtKalman*((MRL - Clong*(100*((omegaRL*tireRadius)/(vx - r*(w/2))-1))*tireRadius)/IRad);
                            omegaRR+dtKalman*((MRR - Clong*(100*((omegaRR*tireRadius)/(vx - r*(-w/2))-1))*tireRadius)/IRad)];  

        F =            [    1, 0, dtKalman*(-vx*sin(phi) - vy*cos(phi)), dtKalman*cos(phi), -dtKalman*sin(phi), 0, 0, 0, 0, 0, 0, 0;
                            0, 1, dtKalman*(vx*cos(phi) - vy*sin(phi)), dtKalman*sin(phi), dtKalman*cos(phi), 0, 0, 0, 0, 0, 0, 0;
                            0, 0, 1, 0, 0, dtKalman, 0, 0, 0, 0, 0, 0;
                            0, 0, 0, 1, dtKalman*r, dtKalman*vy, dtKalman, 0, 0, 0, 0, 0;
                            0, 0, 0, -dtKalman*r, 1, -dtKalman*vx, 0, dtKalman, 0, 0, 0, 0;
                            0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
                            0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
                            0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
                            0, 0, 0, 100*dtKalman*Clong*omegaFL*tireRadius^2/((vx - r*w/2)^2*cos(delta)*IRad), 0, -50*dtKalman*Clong*omegaFL*tireRadius^2*w/((vx - r*w/2)^2*cos(delta)*IRad), 0, 0, 1 - 100*dtKalman*Clong*tireRadius^2/((vx - r*w/2)*cos(delta)*IRad), 0, 0, 0;
                            0, 0, 0, 100*dtKalman*Clong*omegaFR*tireRadius^2/((vx + r*w/2)^2*cos(delta)*IRad), 0, 50*dtKalman*Clong*omegaFR*tireRadius^2*w/((vx + r*w/2)^2*cos(delta)*IRad), 0, 0, 0, 1 - 100*dtKalman*Clong*tireRadius^2/((vx + r*w/2)*cos(delta)*IRad), 0, 0
                            0, 0, 0, 100*dtKalman*Clong*omegaRL*tireRadius^2/((vx - r*w/2)^2*IRad), 0, -50*dtKalman*Clong*omegaRL*tireRadius^2*w/((vx - r*w/2)^2*IRad), 0, 0, 0, 0, 1 - 100*dtKalman*Clong*tireRadius^2/((vx - r*w/2)*IRad), 0;
                            0, 0, 0, 100*dtKalman*Clong*omegaRR*tireRadius^2/((vx + r*w/2)^2*IRad), 0, 50*dtKalman*Clong*omegaRR*tireRadius^2*w/((vx + r*w/2)^2*IRad), 0, 0, 0, 0, 0, 1 - 100*dtKalman*Clong*tireRadius^2/((vx + r*w/2)*IRad)];
        
        %Heading Angle Wrapping after predict step
        S(3) = mod(S(3),2*pi);
        
       %Predict State Covariance
       P = F * P * F' + Q;
       P = 1 / 2 * (P + P');

end

function [S,P] = predictDynamicWithoutWheelspeed(S,P,dtKalman,Q)

        %% Prediction
        
        % Write state as letters to easily understand the calculation in the predict step
        X = S(1);
        Y = S(2);
        phi =S(3);
        vx =S(4);
        vy = S(5);
        r = S(6);
        ax =S(7);
        ay =S(8);
        omegaFL =S(9);
        omegaFR =S(10);
        omegaRL =S(11);
        omegaRR =S(12);
        
        
            S =   [         X+dtKalman*(vx*cos(phi) - vy*sin(phi));
                            Y+dtKalman*(vx*sin(phi) + vy*cos(phi));
                            phi+dtKalman*(r);
                            vx+dtKalman*(ax+r*vy);
                            vy+dtKalman*(ay-r*vx);
                            r;
                            ax;
                            ay;
                            omegaFL;
                            omegaFR;
                            omegaRL;
                            omegaRR];
                            

                            

        F =            [    1, 0, dtKalman*(-vx*sin(phi) - vy*cos(phi)), dtKalman*cos(phi), -dtKalman*sin(phi), 0, 0, 0, 0, 0, 0, 0;
                            0, 1, dtKalman*(vx*cos(phi) - vy*sin(phi)), dtKalman*sin(phi), dtKalman*cos(phi), 0, 0, 0, 0, 0, 0, 0;
                            0, 0, 1, 0, 0, dtKalman, 0, 0, 0, 0, 0, 0;
                            0, 0, 0, 1, dtKalman*r, dtKalman*vy, dtKalman, 0, 0, 0, 0, 0;
                            0, 0, 0, -dtKalman*r, 1, -dtKalman*vx, 0, dtKalman, 0, 0, 0, 0;
                            0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
                            0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
                            0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
                            0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1];
        
        %Heading Angle Wrapping after predict step
        S(3) = mod(S(3),2*pi);
        
       %Predict State Covariance
       P = F * P * F' + Q;
       P = 1 / 2 * (P + P');


end

function [S,P] = make_ins_position_measurement(S,P,position_enu_x,position_enu_y)
    
    z = [position_enu_x;position_enu_y];

    R = [1.0000e-04,0;
         0,1.0000e-04];
     
    H = [     1,0,0,0,0,0,0,0,0,0,0,0;
              0,1,0,0,0,0,0,0,0,0,0,0];
          
    diff = z - H * S;
    [S,P] = kalman_correct(S,P,R,H,diff);

end

function [S,P] = make_ins_heading_measurement(S,P,heading_enu_z)
    
    z = [heading_enu_z];

    R = [1.9496e-06];
     
    H = [     0,0,1,0,0,0,0,0,0,0,0,0];
          
    diff = mod((z-(H * S)+pi),2*pi)-pi;
    [S,P] = kalman_correct(S,P,R,H,diff);

end

function [S,P] = make_ins_velocity_vehicle_measurement(S,P,linear_velocity_vehicle_x,linear_velocity_vehicle_y)
    
    z = [linear_velocity_vehicle_x;linear_velocity_vehicle_y];

    R = [1.0000e-04,0;
         0,1.0000e-04];
     
    H = [          0,0,0,1,0,0,0,0,0,0,0,0;
                   0,0,0,0,1,0,0,0,0,0,0,0];
          
    diff = z - H * S;

    [S,P] = kalman_correct(S,P,R,H,diff);

end

function [S,P] = make_imu_measurement(S,P,angular_velocity_vehicle_z,linear_acceleration_vehicle_x,linear_acceleration_vehicle_y)
    
    z = [angular_velocity_vehicle_z;linear_acceleration_vehicle_x;linear_acceleration_vehicle_y];

    R = [1.2185e-11,0   ,0      ;
         0   ,1.0000e-08,0      ;
         0   ,0,   1.0000e-08];
     
    H = [          0,0,0,0,0,1,0,0,0,0,0,0;
                   0,0,0,0,0,0,1,0,0,0,0,0;
                   0,0,0,0,0,0,0,1,0,0,0,0];
          
    diff = z - H * S;

    [S,P] = kalman_correct(S,P,R,H,diff);

end

function [S,P] = make_odometry_measurement(S,P,wheelspeed_front_left_rpm,wheelspeed_front_right_rpm,wheelspeed_rear_left_rpm,wheelspeed_rear_right_rpm)
    
    z = [wheelspeed_front_left_rpm;wheelspeed_front_right_rpm;wheelspeed_rear_left_rpm;wheelspeed_rear_right_rpm];

    R = [   0.033225931846201,0,0,0;
         0,   0.033225931846201,0,0;
         0,0,   0.033225931846201,0
         0,0,0,   0.033225931846201];
     
     
    H = [          0,0,0,0,0,0,0,0,1,0,0,0;
                   0,0,0,0,0,0,0,0,0,1,0,0;
                   0,0,0,0,0,0,0,0,0,0,1,0;
                   0,0,0,0,0,0,0,0,0,0,0,1];
          
    diff = z - H * S;

    [S,P] = kalman_correct(S,P,R,H,diff);

end

function [S,P] = make_gss_measurement(S,P,linear_velocity_vehicle_x,linear_velocity_vehicle_y,x_pos_gss,y_pos_gss)
    
    z = [linear_velocity_vehicle_x;linear_velocity_vehicle_y];

    R = [0.1225,0;
         0,0.1225];
     
    H = [         0,0,0,1,0,-y_pos_gss,0,0,0,0,0,0;
                  0,0,0,0,1,x_pos_gss,0,0,0,0,0,0];
          
    diff = z - H * S;

    [S,P] = kalman_correct(S,P,R,H,diff);


end 

function [S,P] = kalman_correct(S,P,R,H,diff)
   
          
    L = inv((H * P * H' + R));
    
    K = (L'*H*P)';

    S = S + K * diff;
    
    I = eye(height(S));

    G = (I - K * H);
    
    P = G * P * G' + K * R * K';
end
    
