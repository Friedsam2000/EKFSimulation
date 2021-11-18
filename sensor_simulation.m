clear all
close all

fullMatFileName = 'C:\Users\samue\Documents\Git\EKF_Samuel\simulationResults.mat';
if ~exist(fullMatFileName, 'file')
  message = sprintf('Run FourWheelModel.m first to generate Results', fullMatFileName);
  uiwait(warndlg(message));
else
  load(fullMatFileName);
end

%% Set Variances (Standardabweichung = sqrt(Varianz))
XvarGNSS = (0.01)^2; % 1 cm
YvarGNSS = (0.01)^2; % 1 cm
vxvarGNSS = (0.01)^2; % 0.01 m/s
vyvarGNSS = (0.01)^2; % 0.01 m/s
phivarGNSS = (0.08*pi/180)^2;  % 0.08 deg
axvarIMU = (0.0001)^2; % 0.0001 m/s^2
ayvarIMU = (0.0001)^2; % 0.0001 m/s^2
rvarIMU = (0.0002*pi/180)^2; % 0.0002 rad/s
vxvarGSS = (0.35)^2; % 0.35 m/s
vyvarGSS = (0.35)^2; % 0.35 m/s
omegavarWSS = ((20/60/11.49)*2*pi)^2; % 20 rpm 
deltavar = (1*pi/180)^2; % 1 deg 
Mvar = (2)^2; % 2 Nm

%% 

%GNSS
Xtrue = X(1:k);
Ytrue = Y(1:k);
phiTrue = phi(1:k);
XGNSSMeas = Xtrue+sqrt(XvarGNSS)*randn(size(Xtrue));
YGNSSMeas = Ytrue+sqrt(YvarGNSS)*randn(size(Ytrue));
phiGNSSMeas = phiTrue+sqrt(phivarGNSS)*randn(size(phiTrue));
phiGNNSMeas = mod(phiGNSSMeas,2*pi);
for i = 1:length(phiGNSSMeas)
    if phiGNSSMeas(i) < 0
        phiGNSSMeas(i) = 2*pi + phiGNSSMeas(i);
    end
    if phiGNSSMeas(i) > 2*pi
        phiGNSSMeas(i) = 2*pi - phiGNSSMeas(i);
    end
end
vxTrue = vx(1:k);
vyTrue = vy(1:k);
vxGNSSMeas = vxTrue +sqrt(vxvarGNSS)*randn(size(vxTrue));
vyGNSSMeas = vyTrue +sqrt(vyvarGNSS)*randn(size(vyTrue));

%IMU
axTrue = saves.ax(1:k);
ayTrue = saves.ay(1:k);
rTrue = r(1:k);
axIMUMeas = axTrue+sqrt(axvarIMU)*randn(size(axTrue));
ayIMUMeas = ayTrue+sqrt(ayvarIMU)*randn(size(ayTrue));
rIMUMeas = rTrue+sqrt(rvarIMU)*randn(size(rTrue));

%GSS
vxGSSTrue = vxKistler(1:k);
vyGSSTrue = vyKistler(1:k);
vxGSSMeas = vxGSSTrue+sqrt(vxvarGSS)*randn(size(vxGSSTrue));
vyGSSMeas = vyGSSTrue+sqrt(vyvarGSS)*randn(size(vyGSSTrue));

%Wheelspeed
omegaFLTrue = omegaFL(1:k);
omegaFRTrue = omegaFR(1:k);
omegaRLTrue = omegaRL(1:k);
omegaRRTrue = omegaRR(1:k);
omegaFLMeas = omegaFLTrue+sqrt(omegavarWSS)*randn(size(omegaFLTrue));
omegaFRMeas = omegaFRTrue+sqrt(omegavarWSS)*randn(size(omegaFRTrue));
omegaRLMeas = omegaRLTrue+sqrt(omegavarWSS)*randn(size(omegaRLTrue));
omegaRRMeas = omegaRRTrue+sqrt(omegavarWSS)*randn(size(omegaRRTrue));

%Motor Torque
MFL = saves.MFL(1:k) +sqrt(Mvar)*randn(size(saves.MFL(1:k)));
MFR = saves.MFR(1:k) +sqrt(Mvar)*randn(size(saves.MFR(1:k)));
MRL = saves.MRL(1:k) +sqrt(Mvar)*randn(size(saves.MRL(1:k)));
MRR = saves.MRR(1:k) +sqrt(Mvar)*randn(size(saves.MFR(1:k)));

%Steering Angle
deltaTrue = delta(1:k);
deltaMeas = deltaTrue+sqrt(deltavar)*randn(size(deltaTrue)) +(0*pi/180);


plot(vxGSSMeas,'LineWidth',1.5,'Color',[0.8500, 0.3250, 0.0980]);
hold on
plot(vxGSSTrue,'LineWidth',1.5,'Color',[0, 0.4470, 0.7410]);
ylabel('vx [m/s]');
grid on
set(gca,'FontSize',22)
legend('vx wahr','vx optischer Sensor');




save('sensorResults.mat');
