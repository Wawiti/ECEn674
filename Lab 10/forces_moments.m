% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    sp = sin(phi);
    cp = cos(phi);
    st = sin(theta);
    ct = cos(theta);
    sps = sin(psi);
    cps = cos(psi);
    
    Rot_v_b = [ ct*cps ct*sps -st;...
                sp*st*cps-cp*sps sp*st*sps+cp*cps sp*ct;...
                cp*st*cps+sp*sps cp*st*sps+sp*cps cp*ct;];
    
    % compute wind data in NED
    Wind_body = Rot_v_b*[w_ns; w_es; w_ds]+[u_wg; v_wg; w_wg];
    Airspeed_body = [u-Wind_body(1); v-Wind_body(2); w-Wind_body(3)];
    
    % compute wind data in NED
    w_n = 0;
    w_e = 0;
    w_d = 0;
    
    % Airspeed Vector
    u_r = Airspeed_body(1);      % Airspeed x
    v_r = Airspeed_body(2);      % Airspeed y
    w_r = Airspeed_body(3);      % Airspeed z
    
    % compute air data
    Va = sqrt((u_r)^2+(v_r)^2+(w_r)^2);
    alpha = atan(w_r/u_r);
    beta = asin(v_r/Va);
    
    % Coefficients
    sigma = (1 + exp(-P.M * (alpha - P.alpha0)) + exp(P.M * (alpha + P.alpha0))) ...
            / ((1 + exp(-P.M * (alpha - P.alpha0))) * (1 + exp(P.M * (alpha + P.alpha0))));
    CD = P.C_D_p + (P.C_L_0 + P.C_L_alpha * alpha)^2 / (pi * P.e * P.b^2 / P.S_wing);
    CDq = P.C_D_q;
    CDde = P.C_D_delta_e;
    
    CL = (1 - sigma) * (P.C_L_0 + P.C_L_alpha * alpha) + sigma * (2 * sign(alpha) * (sin(alpha))^2 * cos(alpha));
    CLq = P.C_L_q;
    CLde = P.C_L_delta_e;
    
    Cx = -CD*cos(alpha)+CL*sin(alpha);
    Cxq = -CDq*cos(alpha)+CLq*sin(alpha);
    Cxde = -CDde*cos(alpha)+CLde*sin(alpha);
    
    Cz = -CD*sin(alpha)-CL*cos(alpha);
    Czq = -CDq*sin(alpha)-CLq*cos(alpha);
    Czde = -CDde*sin(alpha)-CLde*cos(alpha);
    
    % compute external forces and torques on aircraft
    Force(1) =  -P.mass*P.gravity*sin(theta)+...
                (1/2)*P.rho*Va^2*P.S_wing*(Cx+Cxq*(P.c/(2*Va))*q+Cxde*delta_e)+...
                (1/2)*P.rho*P.S_prop*P.C_prop*((P.k_motor*delta_t)^2-Va^2);
    Force(2) =  P.mass*P.gravity*cos(theta)*sin(phi)+...
                (1/2)*P.rho*Va^2*P.S_wing*(P.C_Y_0+P.C_Y_beta*beta+P.C_Y_p*(P.b/(2*Va))*p+P.C_Y_r*(P.b/(2*Va))*r+P.C_Y_delta_a*delta_a+P.C_Y_delta_r*delta_r);
    Force(3) =  P.mass*P.gravity*cos(theta)*cos(phi)+...
                (1/2)*P.rho*Va^2*P.S_wing*(Cz+Czq*(P.c/(2*Va))*q+Czde*delta_e);
    
    Torque(1) = (1/2)*P.rho*Va^2*P.S_wing*(P.b*(P.C_ell_0+P.C_ell_beta*beta+P.C_ell_p*(P.b/(2*Va))*p+P.C_ell_r*(P.b/(2*Va))*r+P.C_ell_delta_a*delta_a+P.C_ell_delta_r*delta_r))+...
                    -P.k_T_P*(P.k_Omega*delta_t)^2;
    Torque(2) = (1/2)*P.rho*Va^2*P.S_wing*(P.c*(P.C_m_0+P.C_m_alpha*alpha+P.C_m_q*(P.b/(2*Va))*q+P.C_m_delta_e*delta_e));   
    Torque(3) = (1/2)*P.rho*Va^2*P.S_wing*(P.b*(P.C_n_0+P.C_n_beta*beta+P.C_n_p*(P.b/(2*Va))*p+P.C_n_r*(P.b/(2*Va))*r+P.C_n_delta_a*delta_a+P.C_n_delta_r*delta_r));
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



