function Rotation = R_inv(phi, theta, psi)
    arguments
        phi  double
        theta  double
        psi  double
    end
    Rotation = zeros(3,3); % Preallocate to turn off the stupid error
    Rotation = inv( R(phi, theta, psi) );
end