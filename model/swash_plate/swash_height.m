function h = swash_height(theta)
    load("C:\Users\omar_\OneDrive - Imperial College London\EIE Year 4\FYP\model\helicopter_functions\variables.mat");
    h =  sign(theta) * sqrt( r_p^2 - r_s^2 * ( cos(theta) )^2 - 2*r_s*b*cos(theta) - b^2  );

end