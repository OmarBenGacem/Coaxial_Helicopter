clc;
clear;

% Other Paths
addpath("helicopter_functions/")
addpath("swash_plate/")




% Swash Plate
r_s = 0.025;
r_p = 0.032;
b = 0.003;
servo_max = 45;
servo_min = -45;
rotation_plane_radius = 0.05;
servo_angles = [ 0 , 120, 240 ];
save("variables.mat")
neutral_height = [pitch_height(0); pitch_height(0); pitch_height(0)] ;
save("variables.mat")










%%%% JANSENS VARIABLES %%%%

g = 9.81;                   % gravity
helicopter_mass = 2.112;    % mass
I_xx = 55390782.61*10^-9;   % inertia xx
I_yy = 59517964.39*10^-9;   % inertia yy    
I_zz = 18366526.10*10^-9;   % inertia zz
h_z1 = 0.06717;             % height from COG of rotor 1
h_z2 = 0.02212;             % height from COG of rotor 2
k = 1;                      %%%%%% flapping stiffness should go in here and find actual dynamics stiffnes of the carbon fibre at the length I have with assuming ridigid grips
sigma = 0.054;              % Rotor solidity (ratio of rotor to disk area) https://en.wikipedia.org/wiki/Rotor_solidity#:~:text=Rotor%20solidity%20is%20the%20ratio,area%20of%20the%20rotor%20disk.&text=is%20the%20disk%20area.&text=where%3A,corresponding%20to%20the%20blade%20section
lambda = 15;                % Lock Number https://en.wikipedia.org/wiki/Lock_number#:~:text=In%20helicopter%20aerodynamics%2C%20the%20Lock,studied%20autogyros%20in%20the%201920s.
pho = 1.225;                % Air denisty
% a_t = 0.0118;               % thrust curve slope - reference CT_derivation file in this folder equation derviation section
R = 0.714/2;                % Rotor radius

Inertia_Principal = [I_xx, 0, 0 ; 0, I_yy, 0 ; 0, 0, I_zz];





N = 2; % number of blades
Chord = 0.00313; % chord length
sigma = 0.054;
a = 10; % lift curve slope

% starting points of the iteration
h = 0.001;
R = 0.714/2;
r_values = 0:h:R; % values of r
l_values = []; % inflow ratio values
theta = [0:1:10];
CT_values = [];
for theta_h = theta
    for r = r_values
      fun = @(lamda) 0.5*sigma*Chord*a*(theta_h - lamda) - ...
                    4*(2/pi)*acos(exp((N*(r - 1))/(2 * lamda)))*lamda^2;
    
      % Initial guess (adjust if needed)
      x0 = [0.05];
    
      % Solve for l using fsolve
      l = fsolve(fun, x0, optimset('Display','off'));
    
      % Append the calculated l to the l_values array
      l_values = [l_values, l];
    end

    % calculating CT using the trapezium rule summantation
    A = l_values(1);
    B = l_values(length(l_values));
    f_x = l_values(2:length(l_values)-1);

    CT = h * (A/2 + sum(f_x) + B/2);
    CT_values = [CT_values, CT];
end
clear fun


%% rotor forces and touques


Mrpm = [-48.3506241781727, 12.5197408660271 ; -48.3506241781727, -12.5197408660271];
Mcyc_I = [0,5.165602770199550;4.807401984027046,0];
Mcyc_u = [0,15.685964650737061;14.598245536487191,0];
Mcol = [-0.161168747260932,0.027821646369348;-0.161168747260932,-0.027821646369349];

% RPM_radians = @(RPM) RPM*2*pi/60;


% generate_thrust_coeff = @(xq) interp1(theta, CT_values, xq, 'spline', 'extrap');
% 
% 
% Thrust_Coefficient_Top = @(heave_angle) heave_angle * generate_thrust_coeff(heave_angle);
% Thrust_Coefficient_Bot = @(heave_angle) Thrust_Coefficient_Top(heave_angle);
% 
% Torque_Coefficient_Top = @(heave_angle) Thrust_Coefficient_Top(heave_angle) * sqrt( Thrust_Coefficient_Top(heave_angle)/2 );
% Torque_Coefficient_Bot = @(heave_angle) Torque_Coefficient_Top(heave_angle);
% 
% 
% 
% Thrust = @(heave_angle, RPM)  generate_thrust_coeff(heave_angle)*pho*pi*R^4*RPM^2;
% 
% Drag = @(heave_angle, RPM)  (enerate_thrust_coeff(heave_angle).*sqrt(enerate_thrust_coeff(heave_angle)/2))*pho*pi*R^5*RPM^2;






%% Flapping

Bc1 = -k;
% Bc1_u = @(heave_angle) -8 * (2 * Thrust_Coefficient_Top(heave_angle) /sigma + sqrt(Thrust_Coefficient_Top(heave_angle)/2)/4);
% Bc1_q = @(heave_angle) 16/lambda - Bc1_u(heave_angle)  *h_z1;
Bc1_p = 1;


Bs1 = k;
% Bs1_v = Bc1_u;
% Bs1_p = @(heave_angle) - Bc1_q(heave_angle);
Bs1_q = 1;


Bc2 = -k;
% Bc2_u = Bc1_u;
% Bc2_q = @(heave_angle) 16/lambda - Bc1_u(heave_angle)*h_z2;
Bc2_p = -1;


Bs2 = -k;
% Bs2_v = @(heave_angle) -Bc1_u(heave_angle);
% Bs2_p = @(heave_angle) Bc2_q(heave_angle) ; %%% check that this should not be negative in proper helicopter book
Bs2_q = 1;


% lower flapping coeffiecnts
% B_cl = @(heave_angle) theta_lc*Bc1 + Bc1_u(heave_angle)*u + Bc1_q(heave_angle)*q + Bc1_p*p;
% B_sl = @(heave_angle) theta_ls*Bs1 + Bs1_v(heave_angle)*v + Bs1_p(heave_angle)*p + Bs1_q*q;

% upper flapping coeffiecents
% B_cu = @(heave_angle) theta_uc*Bc2 + Bc2_u*u + Bc2_q(heave_angle)*q + Bc2_p*p;
% B_su = @(heave_angle) theta_us*Bs2 + Bs2_v*v + Bs2_p(heave_angle)*p + Bs2_q*q;



%% All Functions to Strings

%  RPM_radians= func2str(RPM_radians);
% generate_thrust_coeff = func2str(generate_thrust_coeff);
%  Thrust_Coefficient_Top= func2str(Thrust_Coefficient_Top);
%  Thrust_Coefficient_Bot= func2str(Thrust_Coefficient_Bot);
%  Torque_Coefficient_Top= func2str(Torque_Coefficient_Top);
%  Torque_Coefficient_Bot= func2str(Torque_Coefficient_Bot);
%  Thrust= func2str(Thrust);
%  Drag = func2str(Drag);
%  Bc1_u= func2str(Bc1_u);
%  Bc1_q = func2str(Bc1_q);
%  Bs1_p = func2str(Bs1_p);
%  Bc2_q = func2str(Bc2_q);
%  Bs2_v = func2str(Bs2_v);
%  Bs2_p = func2str(Bs2_p);
%  B_cl = func2str(B_cl);
%  B_sl = func2str(B_sl);
%  B_cu = func2str(B_cu);
%  B_su = func2str(B_su);




save("variables.mat")