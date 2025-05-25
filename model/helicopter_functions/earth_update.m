function [dx,dy,dz,dphi,dtheta,dpsi,dp,dq,dr,du,dv,dw] = earth_update(x,y,z,phi,theta,psi,p,q,r,u,v,w,heave,roll,yaw)

    inverse = R_inv(x.phi, x.theta, x.psi);
    [u_rotated, v_rotated, w_rotated] = inverse * [u,v,w];

    [dx,dy,dz,dphi,dtheta,dpsi,dp,dq,dr,du,dv,dw] = [ u_rotated, v_rotated, w_rotated, p, q, r, p, q, r, u, v, w] ; 


end