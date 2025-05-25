function thrust_vector = thrust_moments(heave, roll, pitch, rpm_top, rpm_bot)
    % v = 1;
    % u = 1;
    % w = 1;
    % p = 1;
    % q = 1;
    % r = 1;
    % % replace later with velocity
    use_flap = false;

    %% COS = ROLL, SIN = PITCH

    % Should return thrust in the linear axial direction
    % u = struct(w_top, w_bot, heave_angle, roll_angle, pitch_angle)
    
    var = load("variables.mat");
    RPM_radians = @(RPM) RPM*2*pi/60;
    generate_thrust_coeff = @(xq) interp1(var.theta, var.CT_values, xq, 'spline', 'extrap');
    Thrust_Coefficient_Top = @(heave_angle) heave_angle * generate_thrust_coeff(heave_angle);
    Thrust_Coefficient_Bot = @(heave_angle) Thrust_Coefficient_Top(heave_angle);
    Torque_Coefficient_Top = @(heave_angle) Thrust_Coefficient_Top(heave_angle) * sqrt( Thrust_Coefficient_Top(heave_angle)/2 );
    Torque_Coefficient_Bot = @(heave_angle) Torque_Coefficient_Top(heave_angle);
    Thrust = @(heave_angle, RPM)  generate_thrust_coeff(heave_angle)*var.pho*pi*var.R^4*RPM^2;  



    if use_flap

        % Drag = @(heave_angle, RPM)  (generate_thrust_coeff(heave_angle).*sqrt(generate_thrust_coeff(heave_angle)/2))*var.pho*pi*var.R^5*RPM^2;
        % Bc1 = -var.k;
        % Bc1_u = @(heave_angle) -8 * (2 * Thrust_Coefficient_Top(heave_angle) /var.sigma + sqrt(Thrust_Coefficient_Top(heave_angle)/2)/4);
        % Bc1_q = @(heave_angle) 16/var.lambda - Bc1_u(heave_angle)  *var.h_z1;
        % Bc1_p = 1;
        % 
        % 
        % Bs1 = var.k;
        % Bs1_v = Bc1_u;
        % Bs1_p = @(heave_angle) - Bc1_q(heave_angle);
        % Bs1_q = 1;
        % 
        % 
        % Bc2 = -var.k;
        % Bc2_u = Bc1_u;
        % Bc2_q = @(heave_angle) 16/var.lambda - Bc1_u(heave_angle)*var.h_z2;
        % Bc2_p = -1;
        % 
        % 
        % Bs2 = -var.k;
        % Bs2_v = @(heave_angle) -Bc1_u(heave_angle);
        % Bs2_p = @(heave_angle) Bc2_q(heave_angle) ; %%% check that this should not be negative in proper helicopter book
        % Bs2_q = 1;
        % 
        % 
        % % lower flapping coeffiecnts
        % B_cl = @(heave_angle, swash_roll) swash_roll*Bc1 + Bc1_u(heave_angle)*u + Bc1_q(heave_angle)*q + Bc1_p*p;
        % B_sl = @(heave_angle, swash_pitch) swash_pitch*Bs1 + Bs1_v(heave_angle)*v + Bs1_p(heave_angle)*p + Bs1_q*q;
        % 
        % % upper flapping coeffiecents
        % B_cu = @(heave_angle, swash_roll) swash_roll*Bc2 + Bc2_u(heave_angle)*u + Bc2_q(heave_angle)*q + Bc2_p*p;
        % B_su = @(heave_angle, swash_pitch) swash_pitch*Bs2 + Bs2_v(heave_angle)*v + Bs2_p(heave_angle)*p + Bs2_q*q;
    
        M_roll = Thrust(heave, rpm_bot) * roll;
        M_pitch = Thrust(heave, rpm_bot) * pitch;
        M_yaw = rpm_top - rpm_bot;

    else
        
        % M_roll = var.h_z1 * Thrust(heave, rpm_bot)*sin(B_sl(heave, pitch)) - var.h_z2 * Thrust(heave, rpm_top) * sin(B_su(heave, pitch));
        % M_pitch = var.h_z1 * Thrust(heave, rpm_top) * cos(B_sl(heave, pitch)) * sin(B_cl(heave, roll)) + var.h_z2 * Thrust(heave, rpm_bot) * cos(B_su(heave, pitch)) * sin(B_cu(heave, roll));
        % M_yaw = rpm_top - rpm_bot;


        M_roll = (var.h_z1 + var.h_z2) * Thrust(heave, rpm_bot) * sin(roll);
        M_pitch = (var.h_z1 + var.h_z2)*Thrust(heave, rpm_bot) * sin(pitch);
        M_yaw = rpm_top - rpm_bot;
    
    end







    


    



    thrust_vector = [M_roll, M_pitch, M_yaw];


end