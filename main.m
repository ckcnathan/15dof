clear
clc
%%
addpath('tires')

%\\\\\ CHASSIS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
front_track = 1.2192;   % front track [m]
rear_track = 1.1684;    % rear track [m] 
wheelbase = 1.575;      % wheel base [m]
car.wdl = 0.5;  % left weight distribution
car.wdf = 0.48; % front weight distribution

car.r.fr = front_track*(1-car.wdl);    %front half track right [m]
car.r.fl = front_track*car.wdl;        %front half track left [m]
car.r.rr = rear_track*(1-car.wdl);    %rear half track right[m]
car.r.rl = rear_track*(car.wdl);      %rear half track left [m]
car.a = wheelbase*(1-car.wdf);    % cg to front axle [m]
car.b = wheelbase*(car.wdf);  % cg to rear axle


car.Ixxf = 80;  % moment of inertia front roll [kgm^2]
car.Ixxr = 80;  % moment of inertia rear roll [kgm^2]
car.Iyy = 95;   % moment of inertia pitch [kgm^2]
car.Izz = 95;    % yaw moment of inertia [kgm^2]
car.cgh = .3;  % CG height [m]

car.m = 301;   % total carmass [kg]
car.mufr = 13;  %unsprung mass front right [kg]
car.mufl = 13;  %unsprung mass front left [kg] 
car.murl = 13;  %unsprung mass rear right[kg]
car.murr = 13;  %unsprung mass rear left [kg]
car.ms = car.m-car.mufr-car.mufl-car.murl-car.murr; % sprung mass [kg]

car.cts = 97402.8; %Chassis torsional stiffness [Nm/rad]	97402.8

%\\\\\ SUSPENSION \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
Kf = 26269;   %spring rate front   [N/m]
cf = 2762.52;   %damper rate front [Ns/m] 2762.52
Kr = 35025.4;   %spring rate front   [N/m] 
cr = 2762.52;   %damper rate front [Ns/m]

Kt = 113832; %tire spring rate  [N/m]
ct = 2782;  %tire damping rate [Ns/m]

car.mr.f = 1.147;    %motion ratio front
car.mr.r = 1.053;    %motion ratio rear

car.K.fl = Kf/(car.mr.f^2);   % wheel rate front left [N/m]
car.K.fr = Kf/(car.mr.f^2);   % wheel rate front right [N/m]
car.K.rl = Kr/(car.mr.r^2);   % wheel rate rear left [N/m]
car.K.rr = Kr/(car.mr.r^2);   % wheel rate rear right [N/m]
car.c.fl = cf/(car.mr.f^2);   % damping at wheel front left [Ns/m]
car.c.fr = cf/(car.mr.f^2);   % damping at wheel front left [Ns/m]
car.c.rl = cr/(car.mr.r^2);   % damping at wheel front left [Ns/m]
car.c.rr = cr/(car.mr.r^2);   % damping at wheel front left [Ns/m]
car.K.t = Kt;   %tire spring rate [N/m]
car.c.t = ct;   % tire damping [Ns/m]

car.rch.f = 0.00994;    % roll center height front [m]
car.rch.r = 0.0144;   % roll center height rear [m]

car.arb.f.mr = 2; % arb motion ratio front
car.arb.r.mr = 2;
car.arb.f.lever = 0.07; % arb lever arm front [m]
car.arb.r.lever = 0.07; 
car.arb.f.TS = 450;  % ARB torsional stiffness front [Nm/rad]
car.arb.r.TS = 0;

%\\\\\ WHEELS & TIRES \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

car.tr.fr = 0.2032; %tire radius front right [m]
car.tr.fl = 0.2032; %tire radius front left [m]
car.tr.rl = 0.2032; %tire radius rear left [m]
car.tr.rr = 0.2032; %tire radius rear right [m]

%tire = tiredata('HOOSIER_20.0x7.5_13_R25B_8rim.mat',2,1,1);
tire = tiredata('HOOSIER_16.0x7.5-10_R20_7rim',2,1,1);
tire.scale = 0.6;   % tire grip scaling factor

%\\\\\ DRIVETRAIN \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
car.Gr = 4.6;   %gear ratio
car.motor = [0 500 1000 1500 2000 2500 3000 3500 4000 4500 5000 5500 6000 6500 7000; 140 147 148 148 148 148 148 148 148 148 148 148 135 68 0];
car.dtmoi = 0.0512*2;   % drivetrain moment of inertia [kgm^2]


%\\\\ STEERING \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
front_toe = 0;  % front toe [deg]
rear_toe = 0;   % rear toe [deg]

car.toe.fl = -front_toe;
car.toe.fr = front_toe;
car.toe.rl = -rear_toe;
car.toe.rr = rear_toe;

car.TC.f = 0;   % front compliance [deg/lateral_g]
car.TC.r = 0;   % rear compliance [deg/lateral_g]

%\\\\ BRAKES \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

car.brakebias = 0.6; %brakebias front


%\\\\ INITIAL CONDITIONS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
car.g = 9.81;   %gravitational acceleration directionless

x0 = 0;
y0 = 0;
phi0 = 0;
xdot0 = 10;
ydot0 = 0;
phidot0 = 0;
theta_dot_fr0 = xdot0*cos(car.toe.fr*pi/180)/car.tr.fr;
theta_dot_fl0 = xdot0*cos(car.toe.fl*pi/180)/car.tr.fl;
theta_dot_rl0 = xdot0*cos(car.toe.rl*pi/180)/car.tr.rl;
theta_dot_rr0 = xdot0*cos(car.toe.rr*pi/180)/car.tr.rr;

Zs0 = 0;
Zsdot0 = 0;
psif0 = 0;
psifdot0 = 0;
psir0 = 0;
psirdot0 = 0;
rho0 = 0;
rhodot0 = 0;

Zufl0 = 0;
Zufldot0 = 0;
Zufr0 = 0;
Zufrdot0 = 0;
Zurl0 = 0;
Zurldot0 = 0;
Zurr0 = 0;
Zurrdot0 = 0;
