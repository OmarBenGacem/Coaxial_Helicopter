function Rotation = R(phi, theta, psi)
    arguments
        phi  double
        theta  double
        psi  double
    end
    Rotation = zeros(3,3); % Preallocate to turn off the stupid error
    % Rotation = [ cos(theta)*cos(psi),  cos(theta)*sin(psi), -sin(theta); ...
    %              sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),  sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), sin(phi)*cos(theta); ...
    %              cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi),  cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*cos(theta)];
    
    

    % Define the rotation matrix R
    Rotation = [ cos(theta)*cos(psi),  sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi),  cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
          cos(theta)*sin(psi),  sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi),  cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
         -sin(theta),           sin(phi)*cos(theta),                             cos(phi)*cos(theta)];

end