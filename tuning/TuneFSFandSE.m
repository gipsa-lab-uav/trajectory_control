clear all;
clc;
dt = 0.01; % Control loop rate in s

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Control of position, speed, uncertainties with Feed Forward %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Physical model: Position, Speed, uncertainties analog to accelerations
% Corresponds to State Representation X_dot = AX + BU & Y = CX + DU ; 
A = [1 dt 0
     0  1 dt
     0  0 1];
 
B = [0 dt 0]';

C = [1 0 0];

% LQR Tuning of parameters = prioritize a part of the control.
% Uncertainties value is always 0 and gain always 1: we want to reject them
aggressivity = 10; % from 0.x to 100, higher means more agressive
positionValue = 1; % should always be 1 as a reference: everything is relative
positionToSpeedRatio = 10; % from 0.01 to 100, higher means prioritize speed
sumRatios = positionValue+positionToSpeedRatio;

% Map userfriendly parameters to LQR parameters for control
positionValue = positionValue/sumRatios;
speedValue = positionToSpeedRatio/sumRatios;

Q = [positionValue 0         0
      0         speedValue   0
      0            0         0]; 
  
% global Q/R rises means gains increase => Higher R = less agressive
R = 1/aggressivity;

K = mydlqr(A,B,Q,R);
Kp = K(1)
Ks = K(2)
Ku = K(3)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Estimation of position, speed & uncertainties %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Physical model: Position, Speed, uncertainties analog to accelerations
% Corresponds to State Representation X_dot = AX + BU & Y = CX + DU ; 
A = [1 dt 0
     0  1 dt
     0  0 1];

C = [1 0 0];

% Map userfriendly parameters to LQR parameters for observation
sensorTrust = 100; % from 0.001 to 1000, higher means you trust the sensor more
% All from 0.001 to 1000, higher means you trust the corrections more than the model
positionModelTrust = 1;% should be always 1 here, or sensorTrust doesn't make sense
speedModelTrust = 10; %relative to position
uncertaintiesModelTrust = 100; %relative to position
sumTrusts = positionModelTrust + speedModelTrust + uncertaintiesModelTrust;

positionGain = positionModelTrust / sumTrusts;
speedGain = speedModelTrust / sumTrusts;
uncertaintiesGain = uncertaintiesModelTrust / sumTrusts;

Robs = 1/sensorTrust; % Robs rises means less trust in the sensor

Qobs = [positionGain 0        0
        0       speedGain     0
        0            0 uncertaintiesGain];

L = (mydlqr(A',C',Qobs,Robs));
Lp = L(1)
Ls = L(2)
Lu = L(3)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Control of position, speed, acceleration & uncertainties %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Physical model: Position, Speed, Acceleration, uncertainties analog to accelerations
% Corresponds to State Representation X_dot = AX + BU (& Y = CX + DU) ; 
Aacc = [1 dt 0  0
        0 1  dt dt
        0 0  1  0
        0 0  0  1];
 
Bacc = [0 dt 0 0]';

% LQR Tuning of parameters = prioritize a part of the control.
% Uncertainties value is always 0 and gain always 1: we want to reject them
aggressivity = 10; % from 0.x to 100, higher means more agressive
positionValue = 1; % should always be 1 as a reference: everything is relative
positionToSpeedRatio = 10; % from 0.01 to 100, higher means prioritize speed
positionToAccRatio = 1; % from 0.01 to 100, higher means prioritize acceleration
sumRatios = positionValue+positionToSpeedRatio+positionToAccRatio;

% Map userfriendly parameters to LQR parameters for control
positionValue = positionValue/sumRatios;
speedValue = positionToSpeedRatio/sumRatios;
accValue = positionToAccRatio/sumRatios;

Qacc = [positionValue 0     0    0
        0        speedValue 0    0
        0             0 accValue 0
        0             0     0    0]; 

% global Q/R rises means gains increase => Higher R = less agressive
Racc = 1/aggressivity;
          
Kacc = mydlqr(Aacc,Bacc,Qacc,Racc);
Kpacc = Kacc(1)
Ksacc = Kacc(2)
Kaacc = Kacc(3)
Kuacc = Kacc(4)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Control of position, speed, acceleration & uncertainties with integrator %%%
%%% This works fine but should probably only be used for Step Controls...    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Physical model: Position, Speed, Acceleration, uncertainties analog to accelerations
% Corresponds to State Representation X_dot = AX + BU (& Y = CX + DU) ; 
Ai = [1 dt 0 0  0
      0 1 dt 0  0
      0 0 1  dt dt
      0 0 0  1  0
      0 0 0  0  1];
 
Bi = [0 0 dt 0 0]';

% LQR Tuning of parameters = prioritize a part of the control.
% Uncertainties value is always 0 and gain always 1: we want to reject them
aggressivity = 10; % from 0.x to 100, higher means more agressive
positionValue = 1; % should always be 1 as a reference: everything is relative
positionToIntegerRatio = 1; % from 0.01 to 100, higher means prioritize integrator
positionToSpeedRatio = 10; % from 0.01 to 100, higher means prioritize speed
positionToAccRatio = 1; % from 0.01 to 100, higher means prioritize acceleration
sumRatios = positionValue+positionToIntegerRatio+positionToSpeedRatio+positionToAccRatio;

% Map userfriendly parameters to LQR parameters for control
integerValue = positionToIntegerRatio/sumRatios;
positionValue = positionValue/sumRatios;
speedValue = positionToSpeedRatio/sumRatios;
accValue = positionToAccRatio/sumRatios;

Qi = [integerValue 0      0      0     0
        0   positionValue 0      0     0
        0          0  speedValue 0     0
        0          0      0   accValue 0
        0          0      0      0     1]; 

% global Q/R rises means gains increase => Higher R = less agressive
Ri = 1/aggressivity;
          
Ki = mydlqr(Ai,Bi,Qi,Ri);
Kii = Ki(1)
Kip = Ki(2)
Kis = Ki(3)
Kia = Ki(4)
Kiu = Ki(5)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Control of position, speed & uncertainties with integrator & feedforward %%%
%%% This works fine but should probably only be used for Step Controls...    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Physical model: Position, Speed, Acceleration, uncertainties analog to accelerations
% Corresponds to State Representation X_dot = AX + BU (& Y = CX + DU) ; 
Aiff = [1 dt 0  0
      0 1  dt 0
      0 0  1  dt
      0 0  0  1];
 
Biff = [0 0 dt 0 ]';

% LQR Tuning of parameters = prioritize a part of the control.
% Uncertainties value is always 0 and gain always 1: we want to reject them
aggressivity = 10; % from 0.x to 100, higher means more agressive
positionValue = 1; % should always be 1 as a reference: everything is relative
positionToIntegerRatio = 1; % from 0.01 to 100, higher means prioritize integrator
positionToSpeedRatio = 10; % from 0.01 to 100, higher means prioritize speed
sumRatios = positionValue+positionToIntegerRatio+positionToSpeedRatio;

% Map userfriendly parameters to LQR parameters for control
integerValue = positionToIntegerRatio/sumRatios;
positionValue = positionValue/sumRatios;
speedValue = positionToSpeedRatio/sumRatios;

Qiff = [integerValue 0      0      0
          0   positionValue 0      0
          0          0  speedValue 0
          0          0      0      0]; 

% global Q/R rises means gains increase => Higher R = less agressive
Riff = 1/aggressivity;
          
Kiff = mydlqr(Aiff,Biff,Qiff,Riff);
Kiffi = Kiff(1)
Kiffp = Kiff(2)
Kiffs = Kiff(3)
Kiffu = Kiff(4)