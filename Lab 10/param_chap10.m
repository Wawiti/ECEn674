clear all;
P.gravity = 9.81;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
%physical parameters of airframe
P.mass = 25;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = .1204;
% aerodynamic coefficients
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;

% wind parameters
P.wind_n = -10;   % Steady wind north
P.wind_e = 0;   % Steady wind east
P.wind_d = 0;   % Steady wind down
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06;
P.sigma_v = 1.06;
P.sigma_w = 0.7;

% r1 - r8
P.r = P.Jx*P.Jz-P.Jxz^2;
P.r1 = P.Jxz*(P.Jx-P.Jy+P.Jz)/P.r;
P.r2 = P.Jz*(P.Jz-P.Jy)+P.Jxz^2;
P.r3 = P.Jz/P.r;
P.r4 = P.Jxz/P.r;
P.r5 = (P.Jz-P.Jx)/P.Jy;
P.r6 = P.Jxz/P.Jy;
P.r7 = ((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/P.r;
P.r8 = P.Jx/P.r;

% C parameters on p.62
P.C_p_0 = P.r3 * P.C_ell_0 + P.r4 * P.C_n_0;
P.C_p_beta = P.r3 * P.C_ell_beta + P.r4 * P.C_n_beta;
P.C_p_p = P.r3 * P.C_ell_p + P.r4 * P.C_n_p;
P.C_p_r = P.r3 * P.C_ell_r + P.r4 * P.C_n_r;
P.C_p_delta_a = P.r3 * P.C_ell_delta_a + P.r4 * P.C_n_delta_a;
P.C_p_delta_r = P.r3 * P.C_ell_delta_r + P.r4 * P.C_n_delta_r;
P.C_r_0 = P.r4 * P.C_ell_0 + P.r8 * P.C_n_0;
P.C_r_beta = P.r4 * P.C_ell_beta + P.r8 * P.C_n_beta;
P.C_r_p = P.r4 * P.C_ell_p + P.r8 * P.C_n_p;
P.C_r_r = P.r4 * P.C_ell_r + P.r8 * P.C_n_r;
P.C_r_delta_a = P.r4 * P.C_ell_delta_a + P.r8 * P.C_n_delta_a;
P.C_r_delta_r = P.r4 * P.C_ell_delta_r + P.r8 * P.C_n_delta_r;


% compute trim conditions using 'mavsim_chap5_trim.slx'
P.Va0 = 35;			% Initial (desired) airspeed   
gamma = 0*pi/180;	% Desired Flight Path Angle (radians)
R 	  = Inf;		% Desired Radius use (+) for right handed orbit

					   
P.Ts = 0.01;
P.tau = 0.5;
			

% initial conditions
P.pn0    = 0;   	% initial North position
P.pe0    = 0;   	% initial East position
P.pd0    = -100;   	% initial Down position (negative altitude)
P.u0     = P.Va0;  	% initial velocity along body x-axis
P.v0     = 0;   	% initial velocity along body y-axis
P.w0     = 0;   	% initial velocity along body z-axis
P.phi0   = 0;   	% initial roll angle
P.theta0 = 0;   	% initial pitch angle
P.psi0   = 0;   	% initial yaw angle
P.p0     = 0;   	% initial body frame roll rate
P.q0     = 0;   	% initial body frame pitch rate
P.r0     = 0;   	% initial body frame yaw rate


% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -100;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

% compute different transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P);

% linearize the equations of motion around trim conditions
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);

compute_gains;
P.altitude_take_off_zone = 50;
P.altitude_hold_zone = 10;	 
					 
% sensor parameters
P.sigma_gyro = 0.13;
P.sigma_accel = 0.0025;
P.beta_abs_pres = 0.125;
P.sigma_abs_pres = 0.01;
P.beta_diff_pres = 0.02;
P.sigma_diff_pres = 0.002;

P.Ts_gps = 1;
P.k_gps = 1/1000;
P.sigma_gps_n = 0.21;
P.sigma_gps_e = 0.21;
P.sigma_gps_h = 0.4;
P.sigma_gps_V_g = 0.05;	   

% filter parameters(chapter 8)
P.alpha_lpf_gyro = 0.1;
P.alpha_lpf_static_pres = 0.1;
P.alpha_lpf_diff_pres = 0.1;

P.bias_gyro_x = 0;             % x-gyro bias
P.bias_gyro_y = 0;             % y-gyro bias
P.bias_gyro_z = 0;             % z-gyro bias

% Autopilot Guidance Model Parameters
wn_h = 4;
zeta_h = 0.707;
P.b_hdot = 2 * zeta_h * wn_h;
P.b_h = wn_h^2;

wn_chi = 5.7;
zeta_chi = 0.707;
P.b_chidot = 2 * zeta_chi * wn_chi;
P.b_chi = wn_chi^2;

P.b_va = 10;
P.gamma_max = 45*pi/180;

% Chapter 10 parameters
P.k_path = 0.01;
P.chi_inf = 30*pi/180;
P.k_orbit = 0.01;

