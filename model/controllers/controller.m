function control_vector = controller(x,y,z,psi,theta,phi,p,q,r,u,v,w,target,x_dot,error)
    
    % delete input feedback, not needed (it is not neccessary)
    % Psi = candlestick
    
    % x -> Pitch -> theta
    % y -> Roll  -> phi
    % z -> Yaw   -> psi

    % +ve pitch_controlled = + pitch = - x motion 


    error_x = error(1);
    error_y = error(2);
    error_z = error(3);

    target_x = target(1);
    target_y = target(2);
    target_z = target(3);
    % target_yaw = 0.2;
    target_yaw = 0;
    
    neutral_heave = -0.04;

%     {'K_derrivative_angle'   }
    % {'K_derrivative_velocity'}
    % {'K_proportional_angle'  }
    % {'Kp_heave'              }
    % {'Kp_yaw'                }

    % THESE NEED TO BE IN ALPHABETICAL ORDER

    K_derrivative_angle = 500;
    K_derrivative_velocity = 0.05;
    K_proportional_angle = 40;
    Kp_heave = 0.017;
    Kp_yaw = 10;
    Parameters = [K_derrivative_angle, K_derrivative_velocity, K_proportional_angle, Kp_heave, Kp_yaw];
    save("controllers/controller_constants.mat", "Parameters");

    controller_constants = load("controllers/controller_constants.mat");
    controller_constants = controller_constants.Parameters;
    K_derrivative_angle     = controller_constants(1);
    K_derrivative_velocity  = controller_constants(2);
    K_proportional_angle    = controller_constants(3);
    Kp_heave                = controller_constants(4);
    Kp_yaw                  = controller_constants(5);

    heave_controlled = neutral_heave + Kp_heave * (target_z - z);
    vars = load("variables.mat");
    RPM_radians = @(RPM) RPM*2*pi/60;
    generate_thrust_coeff = @(xq) interp1(vars.theta, vars.CT_values, xq, 'spline', 'extrap');
    Thrust = @(heave_angle, RPM)  generate_thrust_coeff(heave_angle)*vars.pho*pi*vars.R^4*RPM^2;



    
    rpm_top_controlled = 300 + Kp_yaw * ( target_yaw - phi );
    rpm_bottom_controlled = 300 - Kp_yaw * ( target_yaw - phi );
         

    % Compute Errors
    pitch_error = [target_x - x ; theta];
    roll_error = [- (target_y - y) ; psi];



    pitch_controlled = ( -K_proportional_angle * pitch_error(1) + K_derrivative_velocity *  u + q * -K_derrivative_angle);
    roll_controlled  = ( -K_proportional_angle * roll_error(1)  + K_derrivative_velocity * -v + p * -K_derrivative_angle);

    max_heave = 0.15;
    min_heave = -0.15;
    min_pitch = -20;
    max_pitch = 20;
    min_roll = -20;
    max_roll = 20;


    if roll_controlled > max_roll
        roll_controlled = max_roll;
    end
    if roll_controlled < min_roll
        roll_controlled = min_roll;
    end

    if pitch_controlled > max_pitch
        pitch_controlled = max_pitch;
    end
    if pitch_controlled < min_pitch
        pitch_controlled = min_pitch;
    end

    if heave_controlled > max_heave
        heave_controlled = max_heave;
    end
    if heave_controlled < min_heave
        heave_controlled = min_heave;
    end




    roll_controlled = deg2rad(roll_controlled);
    pitch_controlled = deg2rad(pitch_controlled);
    % Delete all of this, just use present measurement (and derrivatives),

    control_vector = [heave_controlled, pitch_controlled, roll_controlled, rpm_top_controlled, rpm_bottom_controlled];

end