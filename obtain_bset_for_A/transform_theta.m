function theta_out=transform_theta(theta_in)
if theta_in>pi
    theta_out=theta_in-2*pi;
else
    theta_out=theta_in;   
end