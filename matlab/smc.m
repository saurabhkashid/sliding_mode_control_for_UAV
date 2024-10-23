clear
clc
syms x x_dot x_ddot y y_dot y_ddot z z_dot z_ddot phi phi_dot pi_ddot theta theta_dot theta_ddot zeta zeta_dot zeta_ddot iy iz ix
syms lambda e e_dot e_ddot q q_dot q_ddot dx dtheta_dot omega k sgnS dz_ddot dz_dot  dy_dot dy dz m u1 g u2 u3 u4 dtheta_ddot ip
syms dphi_ddot dphi_dot dzeta_ddot dzeta_dot
% m= 0.027; l = 0.046; ix = 16.571710*10^-6; iy = ix; iz = 29.261652*10^-6; 
% ip =12.65625810^-8; kf = 1.28192*10^-8; km = 5.964552*10^-3; wmax = 2618; wmin = 0;

%pi = roll, theta = pitch, zeta = yaw

% u1 = m/cos(pi)*cos(theta)() 
ss_dot = ((1/m*(cos(phi)*cos(theta))*u1-g) - dz_ddot)+lambda*(z_dot-dz_dot);
ans1 = solve(ss_dot+k*sgnS==0,u1);

ss_dot2 = ((theta_dot*zeta_dot)*((iy-iz)/ix)-(ip/ix)*omega*theta_dot) + u2/ix - dphi_ddot + lambda*(phi_dot-dphi_dot);
ans2 = solve(ss_dot2+k*sgnS==0,u2);

ss_dot3 = ((phi_dot*zeta_dot)*(iz-ix)/iy+(ip/iy)*omega*phi_dot + u3/iy) - dtheta_ddot + lambda*(theta_dot-dtheta_dot);
ans3 = solve(ss_dot3+k*sgnS==0,u3);

ss_dot4 = ((phi_dot*theta_dot)*(ix-ix)/iz+(u4/iz)) - dzeta_ddot + lambda*(zeta_dot-dzeta_dot);
ans4 = solve(ss_dot4+k*sgnS==0,u4) ;







