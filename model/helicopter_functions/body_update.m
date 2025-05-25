function x_dot = body_update(x,y,z,psi,theta,phi,p,q,r,u,v,w,heave,pitch,roll,rpm_top,rpm_bot)


    state = [x,y,z,psi,theta,phi,p,q,r,u,v,w];
    variables = load("variables.mat", "helicopter_mass", "g", "Inertia_Principal");
    % addpath("helicopter_functions\");
    inverse_rotation = R_inv(psi, theta, phi);
    thrust_vector = (axial_thrust_top(state,heave,roll,pitch,rpm_top,rpm_bot) + axial_thrust_bot(state,heave,roll,pitch,rpm_top,rpm_bot));
    moment_vector = thrust_moments(heave,roll,pitch,rpm_top,rpm_bot);
    gravity_earth = [ 0 ; 0 ; variables.helicopter_mass * variables.g ];
    

    



    %% Body Frame Stuff
    gravity_body = R(psi, theta, phi) * gravity_earth;

    applied_force = thrust_vector' - gravity_body;
    applied_thrust_vector =  applied_force / variables.helicopter_mass;

    du = applied_thrust_vector(1);
    dv = applied_thrust_vector(2);
    dw = applied_thrust_vector(3);
    

    applied_moments_vector = variables.Inertia_Principal * moment_vector';
    dp = applied_moments_vector(1);
    dq = applied_moments_vector(2);  
    dr = applied_moments_vector(3);

    %% Earth Frame Stuff
    
    
    
    global_position = ( (inverse_rotation * [du; dv; dw])' );
    dx = global_position(1);
    dy = global_position(2);
    dz = global_position(3);

    global_rotation = ( (inverse_rotation * [dp; dq; dr])' );
    dpsi =   global_rotation(1); 
    dtheta = global_rotation(2); 
    dphi =   global_rotation(3);
    




    x_dot = [dx,dy,dz,dpsi,dtheta,dphi,dp,dq,dr,du,dv,dw];



end