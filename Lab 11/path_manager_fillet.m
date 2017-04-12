% path_manager_fillet
%   - follow lines between waypoints.  Smooth transition with fillets
%
%
% input is:
%   num_waypoints - number of waypoint configurations
%   waypoints    - an array of dimension 5 by P.size_waypoint_array.
%                - the first num_waypoints rows define waypoint
%                  configurations
%                - format for each waypoint configuration:
%                  [wn, we, wd, dont_care, Va_d]
%                  where the (wn, we, wd) is the NED position of the
%                  waypoint, and Va_d is the desired airspeed along the
%                  path.
%
% output is:
%   flag - if flag==1, follow waypoint path
%          if flag==2, follow orbit
%   
%   Va^d - desired airspeed
%   r    - inertial position of start of waypoint path
%   q    - unit vector that defines inertial direction of waypoint path
%   c    - center of orbit
%   rho  - radius of orbit
%   lambda = direction of orbit (+1 for CW, -1 for CCW)
%
function out = path_manager_fillet(in,P,start_of_simulation)

  NN = 0;
  num_waypoints = in(1+NN);
  waypoints = reshape(in(2+NN:5*P.size_waypoint_array+1+NN),5,P.size_waypoint_array);
  NN = NN + 1 + 5*P.size_waypoint_array;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  % Va      = in(4+NN);
  % alpha   = in(5+NN);
  % beta    = in(6+NN);
  % phi     = in(7+NN);
  % theta   = in(8+NN);
  % chi     = in(9+NN);
  % p       = in(10+NN);
  % q       = in(11+NN);
  % r       = in(12+NN);
  % Vg      = in(13+NN);
  % wn      = in(14+NN);
  % we      = in(15+NN);
  % psi     = in(16+NN);
  state     =  in(1+NN:16+NN);
  NN = NN + 16;
  t         = in(1+NN);
 
  p = [pn; pe; -h];


  persistent waypoints_old   % stored copy of old waypoints
  persistent ptr_a           % waypoint pointer
  persistent ptr_b
  persistent ptr_c
  persistent state_transition % state of transition state machine
  persistent flag_need_new_waypoints % flag that request new waypoints from path planner
  
  
  if start_of_simulation || isempty(waypoints_old),
      waypoints_old = zeros(5,P.size_waypoint_array);
      flag_need_new_waypoints = 0;
     
  end
  
  % if the waypoints have changed, update the waypoint pointer
  if min(min(waypoints==waypoints_old))==0,
      ptr_a = 1;
	  ptr_b = 2;
	  ptr_c = 3;
      waypoints_old = waypoints;
      state_transition = 1;
      flag_need_new_waypoints = 0;
  end
  
  % define current and next two waypoints
	r = waypoints(1:3, ptr_a);
	w = waypoints(1:3, ptr_b);
	w_next = waypoints(1:3, ptr_c);
	q_prev = (w-r)/norm(w-r);
	q_next = (w_next-w)/norm(w_next-w);
	barrho = acos(-q_prev'*q_next);
	
  % define transition state machine
  switch state_transition,
      case 1, % follow straight line from wpp_a to wpp_b
          flag   = 1;  % following straight line path
          Va_d   = waypoints(5,ptr_a); % desired airspeed along waypoint path
          r      = r;
          q      = q_prev;
          q      = q/norm(q);
          c      = [0;0;0;];
          rho    = 1;
          lambda = 1;
		  z = w-(P.R_min/tan(barrho/2))*q_prev;
		  if (p-z)'*q_prev >= 0
			state_transition = 2;
		  end
          
             
      case 2, % follow orbit from wpp_a-wpp_b to wpp_b-wpp_c
          flag   = 2;  % following orbit
          Va_d   = waypoints(5,ptr_a); % desired airspeed along waypoint path
          r      = r;
          q      = q_prev;
          q_next = q_next;
          c      = w - (P.R_min/sin(barrho/2))*(q_prev-q_next)/norm(q_prev-q_next);
          rho    = P.R_min;
          lambda = sign(q_prev(1)*q_next(2) - q_prev(2)*q_next(1));
          z = w + (P.R_min/tan(barrho/2))*q_next;
          if (p-z)'*q_next >= 0
             state_transition = 1;
             ptr_a = ptr_a + 1;
             ptr_b = ptr_b + 1;
             ptr_c = ptr_c + 1;
          end
          if ptr_c == 5
             ptr_c = 1;
          end
          if ptr_b == 5
              ptr_b = 1;
          end
          if ptr_a == 5
              ptr_a = 1;
          end

  end
  
  out = [flag; Va_d; r; q; c; rho; lambda; state; flag_need_new_waypoints];

end