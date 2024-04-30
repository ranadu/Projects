%% Project PID values
load final_testing1 % Our matlab code from lab 3 basically Heli 2 Parameters
Mh = 1.422;
Mc = 1.916;

inx = 0.0254; % 1 inch = 0.0254 metres
La = 25.75*inx; %in meters
Lb = 17.688*inx; % in meters
% Polar Moment of Inertia
Je = (Mh*La^2+Mc*Lb^2)*1.05;

% Ft (force required to maintain steady level flight (SLF))
g = 9.81;
Wh = Mh*g;
Wc = Mc*g;
Ft=((La*Wh)-(Lb*Wc))/La;
%Vsum
Kf=0.140;
Vsum=Ft/Kf;
Vsum_new = 21.6;

%% Elevation Simulation
% Transfer function for elevation
elev_num = La;
elev_denom = Je;
G2_elev = tf([elev_num],[elev_denom,0,0])
% sisotool(G2_elev)
time1 = transpose(stockData.time(1:7700,:)); %values for time
volts_elevation = transpose(stockVolts.signals.values(1:7700, 4));
%volts values
elevation1 = transpose(stockData.signals.values(1:7700,2)); %elevation
values
% G2_elev_est = tfest(data_e,2,0) %estimating elevation tf from simulink
% Figure
figure (1)
hold on
plot(time1, elevation1)
hold off
title('Elevation versus Time')
xlabel('Time (s)')
ylabel('Elevation (rad)')
legend('Data1')
% sisotool is used for G2_elev
% sisotool('rlocus', G2_elev);
% C1_PID=pid(C1);
% Kpc1=C1_PID.Kp; Kic1=C1_PID.Ki; Kdc2=C1_PID.Kd;

%% Pitch Simulation
%Finding Moment of Inertia for Pitch
Fw = 0.0025; %Motor-prop force/speed Constant [N/(rad/s)]
Lh = 0.178; %Distance between propellers and pitch axis in meters
Kb = 0.019; % Value obtained from motor prop data is higher than
accepted range
Jp = (Lh^2*Mh)*1.05; % To compensate for assuming straight bar and no
other components, 5% added)
num_pitch = (Fw/Kb)*Lh;
G2_pitch = tf([num_pitch],[Jp 0 0])
% sisotool(G2_pitch)
time1 = transpose(stockData.time(1:7700,:)); %values for time
volts_pitch = transpose(pitchVolts.signals.values(1:7700, 4)); %volts
values for pitch
pitch1 = transpose(pitchData.signals.values(1:7700,2)); %pitch values
% G2_pitch_est = tfest(data_p,2,0)
figure (2)
hold on
plot(time1, pitch1)
hold off
title('Pitch versus Time for heli')
xlabel('Time (s)')
ylabel('Pitch (rad)')
legend('Data1','Data2')
step(G2_trav)

%% Travel Simulation
sisotool(G2_trav)
J_t = Je; % Moment of inertia is same for travel and elevation axis.
Krt = 0.0206;
G2_trav = tf([-Krt], [J_t 0 0])
time1 = transpose(stockData.time(1:7700,:)); %values for time
volts_travel = transpose(trav1Volts.signals.values(1:7700, 4)); %volts
values
travel1 = transpose(trav1Data.signals.values(1:7700,2)); %travel values
% G2_trav_est = tfest(data_t,2,0)
figure (3)
hold on
plot(time1, elevation1)
plot(time1, pitch1)
plot(time1, travel1)
hold off
title('Travel versus Time for Heli 2')
xlabel('Time (s)')
ylabel('Travel (rad)')
legend('Elevation','Pitch','Travel')
