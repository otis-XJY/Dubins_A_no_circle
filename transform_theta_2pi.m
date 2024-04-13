function theta_out=transform_theta_2pi(theta_in)
if theta_in<0
    theta_out=theta_in+2*pi;
else
    theta_out=theta_in;   
end