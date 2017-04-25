function y = autopilot(uu,P)
%
% autopilot for mavsim
% 
% Modification History:
%   2/11/2010 - RWB
%   5/14/2010 - RWB
%   9/30/2014 - RWB
%   

    % process inputs
    NN = 0;
%    pn       = uu(1+NN);  % inertial North position
%    pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    Va       = uu(4+NN);  % airspeed
    alpha    = uu(5+NN);  % angle of attack
    beta     = uu(6+NN);  % side slip angle
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    chi      = uu(9+NN);  % course angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
%    Vg       = uu(13+NN); % ground speed
%    wn       = uu(14+NN); % wind North
%    we       = uu(15+NN); % wind East
%    psi      = uu(16+NN); % heading
%    bx       = uu(17+NN); % x-gyro bias
%    by       = uu(18+NN); % y-gyro bias
%    bz       = uu(19+NN); % z-gyro bias
    NN = NN+19;
    Va_c     = uu(1+NN);  % commanded airspeed (m/s)
    h_c      = uu(2+NN);  % commanded altitude (m)
    chi_c    = uu(3+NN);  % commanded course (rad)
    NN = NN+3;
    t        = uu(1+NN);   % time
    
    autopilot_version = 2;
        % autopilot_version == 1 <- used for tuning
        % autopilot_version == 2 <- LQR Control for longitudinal AP
    switch autopilot_version
        case 1,
           [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 2,
           [delta, x_command] = autopilot_LQR(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
    end
    y = [delta; x_command];
end
    
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot versions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_tuning
%   - used to tune each loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    mode = 1;
    switch mode
        case 1, % tune the roll loop
            phi_c = chi_c; % interpret chi_c to autopilot as course command
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 2, % tune the course loop
            phi_c   = course_hold(chi_c, chi, r, 1, P);             
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 3, % tune the throttle to airspeed loop and pitch loop simultaneously
            theta_c = 20*pi/180 + h_c;
            chi_c = 0;
            phi_c   = course_hold(chi_c, chi, r, 1, P);
            delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 4, % tune the pitch to airspeed loop 
            chi_c = 0;
            delta_t = P.u_trim(4);
            phi_c   = course_hold(chi_c, chi, r, 1, P);
            theta_c = airspeed_with_pitch_hold(Va_c, Va, 1, P);
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 5, % tune the pitch to altitude loop 
            chi_c = 0;
            phi_c   = course_hold(chi_c, chi, r, 1, P);
            theta_c = altitude_hold(h_c, h, 1, P);
            delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
      end
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        theta_c;...
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_LQR
%   - longitudinal autopilot based on Linear Quadratic Regulator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_LQR(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0,
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
        phi_c   = course_hold(chi_c, chi, r, 1, P);

    else
        phi_c   = course_hold(chi_c, chi, r, 0, P);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
    end
    delta_a = roll_hold(phi_c, phi, p, P);       
  
    
    %----------------------------------------------------------
    % longitudinal autopilot based on LQR
    
    output = lqrfunction(Va, q, theta, h, h_c, Va_c, P);
    delta_e = output(1);
    delta_t = output(2);
    theta_c = 0;
 
    delta_e = sat(delta_e, P.delta_e_max, -P.delta_e_max);
    delta_t = sat(delta_t, 0.75, 0);
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
end
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ---------------------------------------------------------
%                 Lateral Autopilot Functions
% --------------------------------------------------------- 
function delta_a = roll_hold(phi_c, phi, p, P)
	persistent integrator;
	persistent error_d1;
% 	if isempty(integrator) % reset (initialize) persistent variables when flag==1
%         integrator = 0;
%         error_d1 = 0;
%     end
    if isempty(error_d1)
        error_d1 = 0;
    end
	error = phi_c - phi;										% Current Error
% 	integrator = integrator + (P.Ts / 2) * (error + error_d1); 	% Update integrator
	error_d1 = error;											% Set error memory
	delta_a = sat(...						% PID Controller
		P.kp_phi * error +...				% Proportional % 		P.ki_phi * integrator -...			% Integral
		P.kd_phi * p,...					% Derivative
		P.delta_a_max, -P.delta_a_max);		% maximum aileron deflection
% 	if P.ki_phi ~= 0						% Keep the integrator from winding up
% 		u_unsat = P.kp_phi * error + P.ki_phi * integrator - P.kd_phi * p;
% 		integrator = integrator + P.Ts / P.ki_phi * (delta_a-u_unsat);
% 	end
end

function phi_c = course_hold(chi_c, chi, r, flag, P)
	persistent integrator;
	persistent error_d1;
	if isempty(integrator) % reset (initialize) persistent variables when flag==1
        integrator = 0;
        error_d1 = 0;
    end
	error = chi_c - chi;										% Current Error
	integrator = integrator + (P.Ts / 2) * (error + error_d1); 	% Update Integrator
	error_d1 = error;											% Set error memory
	phi_c = sat(...							% PID Controller
		P.kp_chi * error +...				% Proportional
		P.ki_chi * integrator,...			% Integral
		P.phi_max, -P.phi_max);				% maximum aileron deflection
	if P.ki_chi ~= 0						% Keep the integrator from winding up
		u_unsat = P.kp_chi * error + P.ki_chi * integrator;
		integrator = integrator + P.Ts / P.ki_chi * (phi_c-u_unsat);
	end
end

% ---------------------------------------------------------
%               Longitudinal Autopilot Functions
% --------------------------------------------------------- 
function output = lqrfunction(Va, q, theta, h, h_c, Va_c,P)
    persistent integrator;
    persistent error_d1;
    if isempty(integrator) % reset (initialize) persistent variables when flag==1
        integrator = [0;0];
        error_d1 = [0;0];
    end
    error = ([h; Va]-[h_c; Va_c]);           % Current Error in Altitude and airspeed
	error(1) = sat(error(1),2,-2);
    error(2) = sat(error(2),2,-2);
    u = (Va-Va_c)*cos(theta);
    w = (Va-Va_c)*sin(theta);
    
    integrator = integrator + (P.Ts / 2) .* (error + error_d1);              % Update Integrator
    error_d1 = error;                                                       % Set error memory
    
    xAug_lon = [u w q theta h-h_c integrator(1) integrator(2)];                 % Augmented x matrix
    
    output = -P.K * xAug_lon';                                              % Solve for output
end


% if(abs(theta) > P.theta_max)
%         % Do nothing, don't change the integrator
%         integrator = integrator - (P.Ts / 2) .* (error + error_d1)              % Update Integrator
%         error_d1 = error;                                                       % Set error memory
%     else
%         % Integrate
%         integrator = integrator + (P.Ts / 2) .* (error + error_d1)              % Update Integrator
%         %integrator = integrator + P.Ts .* error;
%         error_d1 = error;                                                       % Set error memory
%     end
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
  if in > up_limit,
      out = up_limit;
  elseif in < low_limit;
      out = low_limit;
  else
      out = in;
  end
end
  
 
