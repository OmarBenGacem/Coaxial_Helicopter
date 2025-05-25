function height = pitch_height(theta)
    variables = load("variables.mat", "b", "r_p", "r_s");
    r_s = variables.r_s;
    r_p = variables.r_p;
    b = variables.b;

    height = sign( theta ) * sqrt( r_p^2 + - r_s^2*( cos(theta) )^2  - 2*r_s*b*( cos(theta) )  - b^2 );
end