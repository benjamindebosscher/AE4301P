%================================================
%     Matlab Script File used to linearize the 
%     non-linear F-16 model. The program will 
%     Extract the longitudal and lateral 
%     direction matrices.  These system matrices 
%     will be used to create pole-zero mapping
%     and the bode plots of each to each control
%     input.
% Author: Richard S. Russell
% 
%================================================
clear;

addpath obsmutoolsfornewermatlabversions -END % required for some new MATLAB versions

global fi_flag_Simulink

newline = sprintf('\n');

%% Trim aircraft to desired altitude and velocity
%%
% to be replaced by input in final
altitude = 20000 %input('Enter the altitude for the simulation (ft)  :  ');
velocity = 300 %input('Enter the velocity for the simulation (ft/s):  ');

%% Initial guess for trim
%%
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees

%% Find trim for Hifi model at desired altitude and velocity
%%
disp('Trimming High Fidelity Model:');
fi_flag_Simulink = 1;
[trim_state_hi, trim_thrust_hi, trim_control_hi, dLEF, xu_hi] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude);


%% Find the state space model for the hifi model at the desired alt and vel.
%%
trim_state_lin = trim_state_hi; trim_thrust_lin = trim_thrust_hi; trim_control_lin = trim_control_hi;
[A_hi,B_hi,C_hi,D_hi] = linmod('LIN_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3); ...
		dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);

%% Find trim for Hifi model at desired altitude and velocity
%%
disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude);

%% Find the state space model for the hifi model at the desired alt and vel.
%%
trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;
[A_lo,B_lo,C_lo,D_lo] = linmod('LIN_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3);...
		dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);

%% Make state space model
%%
SS_hi = ss(A_hi,B_hi,C_hi,D_hi);
SS_lo = ss(A_lo,B_lo,C_lo,D_lo);


%% Make MATLAB matrix
%%
mat_hi = [A_hi B_hi; C_hi D_hi];
mat_lo = [A_lo B_lo; C_lo D_lo];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Longitudal Directional %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Select the components that make up the longitude A matrix
%%
A_longitude_hi = mat_hi([3 5 7 8 11 13 14], [3 5 7 8 11 13 14]);
A_longitude_lo = mat_lo([3 5 7 8 11 13 14], [3 5 7 8 11 13 14]);

a = 20.2;
A_long_red = zeros(5,5);
A_long_red_ac = A_longitude_lo([3 4 2 5],[3 4 2 5]); %4x4 matrix
A_long_red(1:4,:) = A_longitude_lo([3 4 2 5],[3 4 2 5 7]);
A_long_red(5,:) = [0 0 0 0 -a];
A_long_red_ac_7 = A_longitude_lo([4 5],[4 5]);
%% Select the components that make up the longitude B matrix
%%
B_longitude_hi = mat_hi([3 5 7 8 11 13 14], [19 20]);
B_longitude_lo = mat_lo([3 5 7 8 11 13 14], [19 20]);
B_long_red = [0;0;0;0;a];
B_long_red_ac = A_longitude_lo([3 4 2 5],[7]);
B_long_red_ac_7 = B_long_red_ac([2 4],[1]);

%% Select the components that make up the longitude C matrix
%%
C_longitude_hi = mat_hi([21 23 25 26 29], [3 5 7 8 11 13 14]);
C_longitude_lo = mat_lo([21 23 25 26 29], [3 5 7 8 11 13 14]);

C_long_red = zeros(5,5);
C_long_red_ac = C_longitude_lo([3 4 2 5],[3 4 2 5]);
C_long_red(1:4,1:4) = C_long_red_ac;
C_long_red(5,:) = [0 0 0 0 180/pi];
C_long_red_ac_7 = C_longitude_lo([4 5],[4 5]);
%% Select the components that make up the longitude D matrix
%
D_longitude_hi = mat_hi([21 23 25 26 29], [19 20]);
D_longitude_lo = mat_lo([21 23 25 26 29], [19 20]);
D_long_red = [0;0;0;0;0];
D_long_red_ac = [0;0;0;0];
D_long_red_ac_7 = [0;0];

SS_long_lo_red_ac = ss(A_long_red_ac, B_long_red_ac, C_long_red_ac, D_long_red_ac);%[Vt alpha theta q]
SS_long_lo_red = ss(A_long_red, B_long_red, C_long_red, D_long_red);
SS_long_hi = ss(A_longitude_hi, B_longitude_hi, C_longitude_hi, D_longitude_hi);
SS_long_lo = ss(A_longitude_lo, B_longitude_lo, C_longitude_lo, D_longitude_lo);
SS_long_lo_red_ac_7 = ss(A_long_red_ac_7, B_long_red_ac_7, C_long_red_ac_7, D_long_red_ac_7);

stepplot(-SS_long_lo_red_ac_7([2]),-SS_long_lo_red([4]),20);
legend('2 state model', '4 state model');
ylabel('Pitch rate q [deg/s]');

stepplot(-SS_long_lo_red_ac_7([2]),-SS_long_lo_red([4]),100);
legend('2 state model', '4 state model');
ylabel('Pitch rate q [deg/s]');
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lateral Directional %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Select the components that make up the lateral A matrix
%%
A_lateral_hi = mat_hi([4 6 7 9 10 12 13 15 16], [4 6 7 9 10 12 13 15 16]);
A_lateral_lo = mat_lo([4 6 7 9 10 12 13 15 16], [4 6 7 9 10 12 13 15 16]);


A_lat_red = zeros(6,6);
A_lat_red_ac = A_lateral_lo([4 1 5 6],[4 1 5 6]); %4x4 matrix
A_lat_red(1:4,:) = A_lateral_lo([4 1 5 6],[4 1 5 6 8 9]);
A_lat_red(5,:) = [0 0 0 0 -a 0];
A_lat_red(6,:) = [0 0 0 0 0 -a];

%% Select the components that make up the lateral B matrix
%%
B_lateral_hi = mat_hi([4 6 7 9 10 12 13 15 16], [19 21 22]);
B_lateral_lo = mat_lo([4 6 7 9 10 12 13 15 16], [19 21 22]);

B_lat_red = [0 0;0 0;0 0;0 0;a 0;0 a];
B_lat_red_ac = A_lateral_lo([4 1 5 6],[8 9]);

%% Select the components that make up the lateral C matrix
%%
C_lateral_hi = mat_hi([22 24 25 27 28 30], [4 6 7 9 10 12 13 15 16]);
C_lateral_lo = mat_lo([22 24 25 27 28 30], [4 6 7 9 10 12 13 15 16]);

C_lat_red = zeros(6,6);
C_lat_red_ac = C_lateral_lo([4 1 5 6],[4 1 5 6]);
C_lat_red(1:4,1:4) = C_lat_red_ac;
C_lat_red(5,:) = [0 0 0 0 180/pi 0];
C_lat_red(6,:) = [0 0 0 0 0 180/pi];


%% Select the components that make up the lateral D matrix
%%
D_lateral_hi = mat_hi([22 24 25 27 28 30], [19 21 22]);
D_lateral_lo = mat_lo([22 24 25 27 28 30], [19 21 22]);

D_lat_red_ac = zeros(4,2);
D_lat_red = zeros(6,2);


SS_lat_lo_red = ss(A_lat_red, B_lat_red, C_lat_red, D_lat_red);
SS_lat_lo_red_ac = ss(A_lat_red_ac, B_lat_red_ac, C_lat_red_ac, D_lat_red_ac);%[beta fi p r]
SS_lat_hi = ss(A_lateral_hi, B_lateral_hi, C_lateral_hi, D_lateral_hi);
SS_lat_lo = ss(A_lateral_lo, B_lateral_lo, C_lateral_lo, D_lateral_lo);

%% Make longitudal direction SYSTEM matrix
%%
sys_long_hi = pck(A_longitude_hi, B_longitude_hi, C_longitude_hi, D_longitude_hi);
sys_long_lo = pck(A_longitude_lo, B_longitude_lo, C_longitude_lo, D_longitude_lo);

%% Make lateral direction SYSTEM matrix and Find poles for hifi
%%
sys_lat_hi = pck(A_lateral_hi, B_lateral_hi, C_lateral_hi, D_lateral_hi);

long_poles_hi = spoles(sys_long_hi);
lat_poles_hi = spoles(sys_lat_hi);


%% Make lateral direction SYSTEM matrix and Find poles for lofi
%%
sys_lat_lo = pck(A_lateral_lo, B_lateral_lo, C_lateral_lo, D_lateral_lo);

long_poles_lo = spoles(sys_long_lo);
lat_poles_lo = spoles(sys_lat_lo);


%% Responses & Period and T_1/2
%% 

% frequency and damping ratio could be obtained using damp(sys)
[Wn_lon,zeta_lon,P_lon] = damp(SS_long_lo_red_ac);
[Wn_lat,zeta_lat,P_lat] = damp(SS_lat_lo_red_ac);

% natural frequency
Wn_sp = Wn_lon(4);
Wn_ph = Wn_lon(1);
Wn_spir = Wn_lat(1);
Wn_ap = Wn_lat(2);
Wn_dr = Wn_lat(4);

% damping ratio
zeta_sp = zeta_lon(4)
zeta_ph = zeta_lon(1);
zeta_spir = zeta_lat(1);
zeta_ap = zeta_lat(2);
zeta_dr = zeta_lat(4);

% poles
pole_sp = P_lon(4)
pole_ph = P_lon(1);
pole_spir = P_lat(1);
pole_ap = P_lat(2);
pole_dr = P_lat(4);

% short period
period_sp = 2*pi/abs(imag(pole_sp));
time_to_05_sp = log(0.5)/real(pole_sp);

% phugoid
period_ph = 2*pi/abs(imag(pole_ph));
time_to_05_ph = log(0.5)/real(pole_ph);

% dutch roll
period_dr = 2*pi/abs(imag(pole_dr));
time_to_05_dr = log(0.5)/real(pole_dr);

% aperiodic roll
time_to_05_ap = log(0.5)/real(pole_ap);

% spiral
time_to_05_spir = log(0.5)/real(pole_spir);

% longitudinal responses
[y_lon,t_lon,x_lon] = impulse(SS_long_lo_red_ac);

% lateral responses
[y_lat,t_lat,x_lat] = impulse(SS_lat_lo_red_ac);


plot_switch = 4;
% short period response
if plot_switch == 1
    subplot(4,1,1);
    plot(t_lon(1:30), y_lon(1:30,1))
    % xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('Velocity in [ft/s]', 'FontSize', 12,'FontWeight', 'bold')

    subplot(4,1,2);
    plot(t_lon(1:30), y_lon(1:30,2))
    % xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('\alpha in [deg]', 'FontSize', 12,'FontWeight','bold')

    subplot(4,1,3);1
    plot(t_lon(1:30), y_lon(1:30,3))
    % xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('\theta in [deg]', 'FontSize', 12,'FontWeight', 'bold')

    subplot(4,1,4);
    plot(t_lon(1:30), y_lon(1:30,4))
    xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('q in [deg/s]', 'FontSize', 12,'FontWeight', 'bold')

% phugoid response
elseif plot_switch == 2
    subplot(4,1,1);
    plot(t_lon, y_lon(:,1))
    % xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('Velocity in [ft/s]', 'FontSize', 12,'FontWeight', 'bold')

    subplot(4,1,2);
    plot(t_lon, y_lon(:,2))
    % xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('\alpha in [deg]', 'FontSize', 12,'FontWeight','bold')

    subplot(4,1,3);
    plot(t_lon, y_lon(:,3))
    % xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('\theta in [deg]', 'FontSize', 12,'FontWeight', 'bold')

    subplot(4,1,4);
    plot(t_lon, y_lon(:,4))
    xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('q in [deg/s]', 'FontSize', 12,'FontWeight', 'bold')
 
%dutch roll period response
elseif plot_switch == 3
    subplot(4,1,1);
    plot(t_lat, y_lat(:,1,2))
    % xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('\beta in [deg]', 'FontSize', 12,'FontWeight', 'bold')

    subplot(4,1,2);
    plot(t_lat, y_lat(:,2,2))
    % xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('\phi in [deg]', 'FontSize', 12,'FontWeight','bold')

    subplot(4,1,3);
    plot(t_lat, y_lat(:,3,2))
    % xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('p in [deg/s]', 'FontSize', 12,'FontWeight', 'bold')

    subplot(4,1,4);
    plot(t_lat, y_lat(:,4,2))
    xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('r in [deg/s]', 'FontSize', 12,'FontWeight', 'bold')

% aperiodic roll response
elseif plot_switch == 4
    subplot(4,1,1);
    plot(t_lat, y_lat(:,1,1))
    % xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('\beta in [deg]', 'FontSize', 12,'FontWeight', 'bold')

    subplot(4,1,2);
    plot(t_lat, y_lat(:,2,1))
    % xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('\phi in [deg]', 'FontSize', 12,'FontWeight','bold')

    subplot(4,1,3);
    plot(t_lat, y_lat(:,3,1))
    % xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('p in [deg/s]', 'FontSize', 12,'FontWeight', 'bold')

    subplot(4,1,4);
    plot(t_lat, y_lat(:,4,1))
    xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('r in [deg/s]', 'FontSize', 12,'FontWeight', 'bold')
    
% spiral response
elseif plot_switch == 5
    subplot(4,1,1);
    plot(t_lat, y_lat(:,1,1))
    % xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('\beta in [deg]', 'FontSize', 12,'FontWeight', 'bold')

    subplot(4,1,2);
    plot(t_lat, y_lat(:,2,1))
    % xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('\phi in [deg]', 'FontSize', 12,'FontWeight','bold')

    subplot(4,1,3);
    plot(t_lat, y_lat(:,3,1))
    % xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('p in [deg/s]', 'FontSize', 12,'FontWeight', 'bold')

    subplot(4,1,4);
    plot(t_lat, y_lat(:,4,1))
    xlabel('Time in [s]', 'FontSize', 12,'FontWeight', 'bold')
    ylabel('r in [deg/s]', 'FontSize', 12,'FontWeight', 'bold')
end


%% Pitch rate command controller design task
%%
v_ms = velocity*0.3048;                % velocity in m/s
omega_n_sp = 0.03*v_ms;
time_c = 1/(0.75*omega_n_sp);
damping = 0.5;
g = 9.80665;

% poles
pole_real = -omega_n_sp*damping;
pole_cmpx = omega_n_sp*sqrt(1-damping^2);

poles_placement = [complex(pole_real, pole_cmpx); complex(pole_real, -pole_cmpx)];

K = place(A_long_red_ac_7, B_long_red_ac_7, poles_placement);

% gust check
v_gust = 4.572;                 % gust velocity in m/s
alpha_gust = atan(v_gust/v_ms); % gust angle
d_el_gust = K(1)*alpha_gust;    % elevator deflection angle in case of gust

% CAP and Gibson check
CAP = (omega_n_sp*omega_n_sp)/((v_ms/g)*(1/time_c));        % CAP criterion
GIB = time_c - (2*damping/omega_n_sp);                      % gibson criterion

u0 = zeros(1,1000);
u1 = ones(1,1000);
u = [u1 u0];
t = 0:0.01:19.99;

% placing poles to 2 state system
A_long_red_ac_77 = A_long_red_ac_7 - B_long_red_ac_7*K;
SS_long_lo_red_ac_77 = ss(A_long_red_ac_77, B_long_red_ac_7, C_long_red_ac_7, D_long_red_ac_7);


tf_77 = tf(SS_long_lo_red_ac_77);

s = tf('s');

T = time_c;
desired = (K(2)*(1+T*s))/(s^2 + 2*damping*omega_n_sp*s + omega_n_sp^2);


T1 = -2.183/-0.6606;
K = 1.7087;

prefilter = K*(T1/T)*(T*s+1)/(T1*s+1);
comp = minreal(prefilter*tf_77(2),0.00005);

%% Response of q
%%
[y] = lsim(comp, u, t);
plot(t, y)
hold on 
plot(t, u)
hold off
legend('Time response', 'Step input')
xlabel('Time [s]')
ylabel('Pitch rate q [deg/s]')

%% Response of \theta
%% 
z = 0.01*cumtrapz(lsim(comp, u, t));
plot(t, z)
hold on 
plot(t, u)
hold off
legend('Time response', 'Step input')
xlabel('Time [s]')
ylabel('Pitch attitude angle [deg]')
% tf_77 = ss2tf(A_long_red_ac_77, B_long_red_ac_7, C_long_red_ac_7, D_long_red_ac_7tf_77);

% response of q
%lsim(SS_long_lo_red_ac_77, u, t)


% response of theta in blue
%plot(0.01*cumtrapz(lsim(SS_long_lo_red_ac_77, u, t)))


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Display results


clc;

disp(sprintf('Altitude: %.3f ft.', altitude));
disp(sprintf('Velocity: %.3f ft/s\n\n', velocity));
disp(sprintf('Gust elevator deflection: %.3f degrees\n\n', d_el_gust));

% disp('For HIFI Model:  ');
% disp('Longitudal Direction:  ');
% disp(newline);
% 
% disp('A =')
% for i=1:length( A_longitude_hi(:,1) )
%     mprintf([ A_longitude_hi(i,:) ],'  %.3e ')
% end %for
% 
% disp('B =')
% for i=1:length( B_longitude_hi(:,1) )
%     mprintf([ B_longitude_hi(i,:) ],'  %.3e ')
% end %for
% 
% disp('C =')
% for i=1:length( C_longitude_hi(:,1) )
%     mprintf([ C_longitude_hi(i,:) ],'  %.3e ')
% end %for
% 
% disp('D =')
% for i=1:length( D_longitude_hi(:,1) )
%     mprintf([ D_longitude_hi(i,:) ],'  %.3e ')
% end %for
% 
% rifd(long_poles_hi)
% 
% disp(newline);
% 
% disp('Lateral Direaction:  ');
% 
% disp(newline);
% 
% disp('A =')
% for i=1:length( A_lateral_hi(:,1) )
%     mprintf([ A_lateral_hi(i,:) ],'  %.3e ')
% end %for
% 
% disp('B =')
% for i=1:length( B_lateral_hi(:,1) )
%     mprintf([ B_lateral_hi(i,:) ],'  %.3e ')
% end %for
% 
% disp('C =')
% for i=1:length( C_lateral_hi(:,1) )
%     mprintf([ C_lateral_hi(i,:) ],'  %.3e ')
% end %for
% 
% disp('D =')
% for i=1:length( D_lateral_hi(:,1) )
%     mprintf([ D_lateral_hi(i,:) ],'  %.3e ')
% end %for
% 
% rifd(lat_poles_hi)
% 
% disp(newline);
% disp(newline);
% disp('For LOFI Model:  ');
% disp('Longitudal Direction:  ');
% disp(newline);
% 
% disp('A =')
% for i=1:length( A_longitude_lo(:,1) )
%     mprintf([ A_longitude_lo(i,:) ],'  %.3e ')
% end %for
% 
% disp('B =')
% for i=1:length( B_longitude_lo(:,1) )
%     mprintf([ B_longitude_lo(i,:) ],'  %.3e ')
% end %for
% 
% disp('C =')
% for i=1:length( C_longitude_lo(:,1) )
%     mprintf([ C_longitude_lo(i,:) ],'  %.3e ')
% end %for
% 
% disp('D =')
% for i=1:length( D_longitude_lo(:,1) )
%     mprintf([ D_longitude_lo(i,:) ],'  %.3e ')
% end %for
% 
% % Display the real, imaginary, frequency (magnitude) and damping ratios
% rifd(long_poles_lo)
% 
% disp(newline);
% 
% disp('Lateral Direaction:  ');
% 
% disp(newline);
% 
% disp('A =')
% for i=1:length( A_lateral_lo(:,1) )
%     mprintf([ A_lateral_lo(i,:) ],'  %.3e ')
% end %for
% 
% disp('B =')
% for i=1:length( B_lateral_lo(:,1) )
%     mprintf([ B_lateral_lo(i,:) ],'  %.3e ')
% end %for
% 
% disp('C =')
% for i=1:length( C_lateral_lo(:,1) )
%     mprintf([ C_lateral_lo(i,:) ],'  %.3e ')
% end %for
% 
% disp('D =')
% for i=1:length( D_lateral_lo(:,1) )
%     mprintf([ D_lateral_lo(i,:) ],'  %.3e ')
% end %for

% % Display the real, imaginary, frequency (magnitude) and damping ratios
% rifd(lat_poles_lo)
% 
% %% All Poles
% figure(1); 
% pzmap(SS_hi, 'r', SS_lo, 'b');
% title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nAll Poles\n Blue = lofi Red = hifi.', altitude, velocity);
% title(title_string);
% sgrid;
% 
% %% Long. Poles
% %%
% figure(2); 
% pzmap(SS_long_hi, 'r', SS_long_lo, 'b');
% title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nLongitudal Directional Poles\n Blue = lofi Red = hifi.', altitude, velocity);
% title(title_string);
% sgrid;
% 
% %% Lat. Poles
% %%
% figure(3); 
% pzmap(SS_lat_hi, 'r', SS_lat_lo, 'b');
% title_string = sprintf('Altitude = %.2f ft Velocity = %.2f ft/s\nLateral Directional Poles\n Blue = lofi Red = hifi.', altitude, velocity);
% title(title_string);
% sgrid;
% 
% % Create Bode Plots
% 
% omega = logspace(-2,2,100);
% 
% sysg_lat_hi = frsp(sys_lat_hi,omega);
% sysg_lat_lo = frsp(sys_lat_lo,omega);
% 
% sysg_long_hi = frsp(sys_long_hi,omega);
% sysg_long_lo = frsp(sys_long_lo,omega);
% 
% 
% figure;
% BodeCount = 0;
% for state = 1:1:5
%     for control = 1:1:2
%         BodeCount = BodeCount +1;
%         title_string = sprintf('Bode Plot #%d\n State = %d\n Control = %d', BodeCount,state,control);
%         vplot('bode', sel(sysg_long_hi,state,control), 'b--', sel(sysg_long_lo,state,control), 'r');
%         disp(title_string);
%         legend('hifi', 'lofi');
%         pause;
%     end
% end
% 
% for state = 1:1:6
%     for control = 1:1:3
%         BodeCount = BodeCount + 1;
%         title_string = sprintf('Bode Plot #%d\n State = %d\n Control = %d', BodeCount,state,control);
%         vplot('bode', sel(sysg_lat_hi,state,control), 'b--', sel(sysg_lat_lo,state,control), 'r');
%         disp(title_string);
%         legend('hifi', 'lofi');
%         pause;
%     end
% end