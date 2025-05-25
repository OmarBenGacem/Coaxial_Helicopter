function axial_thrust = axial_thrust(state,heave,roll,pitch,rpm_top,rpm_bot)
    use_flap = false;
    % Should return thrust in the linear axial direction

    x      = state(1);  % x position
    y      = state(2);  % y position
    z      = state(3);  % z position
    phi    = state(4);  % x rotation (roll) 
    theta  = state(5);  % y rotation (pitch)
    psi    = state(6);  % z rotation (yaw)

    p      = state(7);  % x rotational velocity (\dot roll)
    q      = state(8);  % y rotational velocity (\dot pitch)
    r      = state(9);  % z rotational velocity (\dot yaw)
    u      = state(10); % x velocity 
    v      = state(11); % v velocity 
    w      = state(12); % w velocity 


    vars = load("variables.mat");
    RPM_radians = @(RPM) RPM*2*pi/60;
    
    
    generate_thrust_coeff = @(xq) interp1(vars.theta, vars.CT_values, xq, 'spline', 'extrap');
    Thrust = @(heave_angle, RPM)  generate_thrust_coeff(heave_angle)*vars.pho*pi*vars.R^4*RPM^2;

    
    Thrust_Coefficient_Top = @(heave_angle) heave_angle * generate_thrust_coeff(heave_angle);
    Thrust_Coefficient_Bot = @(heave_angle) Thrust_Coefficient_Top(heave_angle);
    Torque_Coefficient_Top = @(heave_angle) Thrust_Coefficient_Top(heave_angle) * sqrt( Thrust_Coefficient_Top(heave_angle)/2 );
    Torque_Coefficient_Bot = @(heave_angle) Torque_Coefficient_Top(heave_angle);

    
    Drag = @(heave_angle, RPM)  (generate_thrust_coeff(heave_angle).*sqrt(generate_thrust_coeff(heave_angle)/2))*vars.pho*pi*vars.R^5*RPM^2;
    Bc1 = -vars.k;
    Bc1_u = @(heave_angle) -8 * (2 * Thrust_Coefficient_Top(heave_angle) /vars.sigma + sqrt(Thrust_Coefficient_Top(heave_angle)/2)/4);
    Bc1_q = @(heave_angle) 16/vars.lambda - Bc1_u(heave_angle)  *vars.h_z1;
    Bc1_p = 1;


    Bs1 = vars.k;
    Bs1_v = Bc1_u;
    Bs1_p = @(heave_angle) - Bc1_q(heave_angle);
    Bs1_q = 1;


    Bc2 = -vars.k;
    Bc2_u = Bc1_u;
    Bc2_q = @(heave_angle) 16/vars.lambda - Bc1_u(heave_angle)*vars.h_z2;
    Bc2_p = -1;


    Bs2 = -vars.k;
    Bs2_v = @(heave_angle) -Bc1_u(heave_angle);
    Bs2_p = @(heave_angle) Bc2_q(heave_angle) ; %%% check that this should not be negative in proper helicopter book
    Bs2_q = 1;


    % lower flapping coeffiecnts
    B_cl = @(heave_angle, roll) deg2rad(roll)*Bc1 + Bc1_u(heave_angle)*u + Bc1_q(heave_angle)*q + Bc1_p*p;
    B_sl = @(heave_angle, pitch) deg2rad(pitch)*Bs1 + Bs1_v(heave_angle)*v + Bs1_p(heave_angle)*p + Bs1_q*q;

    % upper flapping coeffiecents
    B_cu = @(heave_angle, roll) deg2rad(roll)*Bc2 + Bc2_u(heave_angle)*u + Bc2_q(heave_angle)*q + Bc2_p*p;
    B_su = @(heave_angle, pitch) deg2rad(pitch)*Bs2 + Bs2_v(heave_angle)*v + Bs2_p(heave_angle)*p + Bs2_q*q;

    
    
    
    if use_flap
        
        % SINE = Pitch, COSINE = Roll

        % ud = (r_f1*cos(B_sl)*sin(B_cl)  + r_f2*cos(B_su)*sin(B_cu))/m - q*w + r*v;
        % vd = p*w - r*u + (r_f1*sin(B_sl) - r_f2*sin(B_su) + g*m*cos(theta)*sin(phi))/m;
        % wd = q*u - p*v - (r_f1*cos(B_cl)*cos(B_sl) + r_f2*cos(B_cu)*cos(B_su) - g*m*cos(phi)*cos(theta))/m;

        T_x = 0;
        T_y = 0;
        % disp(cos(B_sl(heave, pitch))*sin(B_cl(heave, roll)))
        T_z = ( Thrust(heave, rpm_top)*cos(B_sl(heave, pitch))*sin(B_cl(heave, roll)) + Thrust(heave, rpm_bot)*cos(B_su(heave, pitch))*sin(B_cu(heave, roll)) );

    else
        % Does not generate any thrust about non-axial angles
        T_x = 0;
        T_y = 0;
        % T_z = (rpm_top + rpm_bot);
        T_z = ( Thrust(heave, rpm_top) + Thrust(heave, rpm_bot) );
    end







    axial_thrust = [T_x, T_y, T_z];

end