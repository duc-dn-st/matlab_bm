function A_d_slip = A_d_slip(obj,in2,in3,dt,in5)
%A_D_SLIP
%    A_D_SLIP = A_D_SLIP(OBJ,IN2,IN3,DT,IN5)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    01-Nov-2023 23:07:28

u_lin1 = in3(1,:);
u_lin2 = in3(2,:);
wheel_slip = in5(:,2);
x_lin3 = in2(3,:);
t2 = cos(u_lin2);
t3 = wheel_slip-1.0;
A_d_slip = reshape([1.0,0.0,0.0,0.0,1.0,0.0,dt.*t2.*t3.*u_lin1.*sin(x_lin3),-dt.*t2.*t3.*u_lin1.*cos(x_lin3),1.0],[3,3]);
