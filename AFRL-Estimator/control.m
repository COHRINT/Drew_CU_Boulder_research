classdef control < handle
    methods
        function sys = control(m,Ib,mu,inner_loop_gains)
            sys.m = m;
            sys.Ib = Ib;
            sys.mu = mu;
            % PD attitude control
            % P velocity control
            % Steady-state error exists in velocity term, needs accounted
            % for later, DME: add in I term?
            sys.roll_p = inner_loop_gains(1);
            sys.roll_d = inner_loop_gains(3);
            sys.pitch_p = inner_loop_gains(4);
            sys.pitch_d = inner_loop_gains(6);
            sys.yaw_p = inner_loop_gains(7);
            sys.yaw_d = inner_loop_gains(9);
            sys.velocity_p = inner_loop_gains(10);
            sys.integral_error = [0;0;0];
        end
        function  geometry(sys)
        	% Define 4 HTMs describing propeller location
            thruster1 = [1 0 0 .0707;0 1 0 -.0707;0 0 1 0;0 0 0 1];
            thruster2 = [1 0 0 -.0707;0 1 0 -.0707;0 0 1 0;0 0 0 1];
            thruster3 = [1 0 0 -.0707;0 1 0 .0707;0 0 1 0;0 0 0 1];
            thruster4 = [1 0 0 .0707;0 1 0 .0707;0 0 1 0;0 0 0 1];

            % A matrix converts thrust commands to moments and total thrust
            a = cross(thruster1(1:3,4),[0;0;-1]);
            b = cross(thruster2(1:3,4),[0;0;-1]);
            c = cross(thruster3(1:3,4),[0;0;-1]);
            d = cross(thruster4(1:3,4),[0;0;-1]);
            sys.A = [a b c d];
            sys.A(4,:) = [-1 -1 -1 -1];
            %there is a +/- 10% error in the yaw torque coefficient
            sys.A_actual = sys.A + [0 0 0 0;0 0 0 0;.013*(0.9+rand()/5) -.013*(0.9+rand()/5) .013*(0.9+rand()/5) -.013*(0.9+rand()/5) ;0 0 0 0];
            sys.A_model = sys.A + [0 0 0 0;0 0 0 0;0.13 -0.13 0.13 -0.13; 0 0 0 0];

            % Thrusters 1 and 3 rotate: CCW (posative yaw)
            % Thrusters 2 and 4 rotate: CW (negative yaw)
        end
        function thrust = inner_loop(sys, set_points, State)

            % This function takes the phi, theta, psi and w set points
            % and the current state to determine the desired thrust

            % Calculate attitude error
            roll_err = set_points(1)-State(4);
            pitch_err = set_points(2)-State(5);
            yaw_err = set_points(3)-State(6);
    
            % Calculate rate of change of attitude error
            roll_accel = sys.roll_p*roll_err + sys.roll_d*State(10);
            pitch_accel = sys.pitch_p*pitch_err + sys.pitch_d*State(11);
            yaw_accel = sys.yaw_p*yaw_err + sys.yaw_d*State(12);
    
            % Calculate desired moments
            moments = sys.Ib*[roll_accel;pitch_accel;yaw_accel];

            % Calculate required thrust
            thrust = sys.A_actual\[moments;-set_points(4)];
    
            % Apply thrust limiting (20 N / 4.5 lbf) 
            for j = 1:4
                if thrust(j)>15
                    thrust(j) = 15;
                elseif thrust(j) < 0;
                    thrust(j) = 0;
                end
            end
        end
        function inner_set_points = outer_loop(sys,state, set_points)
            %{
            inner_x_vel_p = .15;
            inner_y_vel_p = -.15;
            %inner_z_vel_p = -.5;
            inner_x_vel_d = 03;
            inner_y_vel_d = 03;
            %}
            proportional_gain = 02.5;
            integral_gain = 0.001;
            derivative_gain = 05; %DME: Changed this from 5 to 0.5 and everything seems to work now

            Or = state(4:6);
            An = state(10:12);
            world_vel = [1 0 0;0 -1 0;0 0 1]*[cos(Or(2))*cos(Or(3)), sin(Or(1))*sin(Or(2))*cos(Or(3))-cos(Or(1))*sin(Or(3)), cos(Or(1))*sin(Or(2))*cos(Or(3))-sin(Or(1))*sin(Or(3));
                  cos(Or(2))*sin(Or(3)), sin(Or(1))*sin(Or(2))*sin(Or(3))-cos(Or(1))*cos(Or(3)), cos(Or(1))*sin(Or(2))*sin(Or(3))-sin(Or(1))*cos(Or(3));
                -sin(Or(2)), sin(Or(1))*cos(Or(2)), cos(Or(1))*cos(Or(2))]*state(7:9);
            
            proportional_error = set_points(1:3)-world_vel;
            % Delta_t should probably be a control property
            sys.integral_error = sys.integral_error + proportional_error*0.01;
            derivative_error = -cross(An,state(7:9));
            
            % Calculate nominal required force required in world frame
            world_force = [set_points(1)*sys.mu;set_points(2)*sys.mu;-9.81*sys.m];
            % Add correctiive control terms
            world_force = world_force + proportional_gain*proportional_error + integral_gain*sys.integral_error + derivative_gain*derivative_error;
            wf_unit = world_force/norm(world_force);
            d = dot([0;0;-1],wf_unit);
            q = acos(d);
            c = cross([0;0;-1],wf_unit);
            c = sin(q/2)*c/norm(c);

            w = cos(q/2);
            x = c(1);
            y = c(2);
            z = c(3);

            w2 = w^2;
            x2 = x^2;
            y2 = y^2;
            z2 = z^2;

            Rd = [w2+x2-y2-z2 2*(x*y-w*z) 2*(w*y+x*z);2*(x*y+w*z) w2-x2+y2-z2 2*(y*z-w*x);2*(x*z-w*y) 2*(w*x+y*z) w2-x2-y2+z2];

            setYaw = Or(3)+0.01*set_points(4);%atan2(Rd(2,1),Rd(1,1));
            setRoll = atan2(Rd(3,2),Rd(3,3));
            setPitch = atan2(-Rd(3,1),Rd(3,3)/cos(setYaw));
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
            %{
            Or = state(4:6);
            An = state(10:12);
            world_vel = [1 0 0;0 -1 0;0 0 1]*[cos(Or(2))*cos(Or(3)), sin(Or(1))*sin(Or(2))*cos(Or(3))-cos(Or(1))*sin(Or(3)), cos(Or(1))*sin(Or(2))*cos(Or(3))-sin(Or(1))*sin(Or(3));
                              cos(Or(2))*sin(Or(3)), sin(Or(1))*sin(Or(2))*sin(Or(3))-cos(Or(1))*cos(Or(3)), cos(Or(1))*sin(Or(2))*sin(Or(3))-sin(Or(1))*cos(Or(3));
                            -sin(Or(2)), sin(Or(1))*cos(Or(2)), cos(Or(1))*cos(Or(2))]*state(7:9);
  
            setRoll = state(5)+inner_y_vel_p*(world_vel(2)-set_points(2));
            setPitch = state(4)+inner_x_vel_p*(world_vel(1)-set_points(1));
            %}
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
            inner_set_points = [setRoll;setPitch;setYaw;norm(world_force)];
        end
    end
    properties
        m
        mu
        Ib
        roll_p
        roll_d
        pitch_p
        pitch_d
        yaw_p
        yaw_d
        velocity_p
        A
        A_actual
        A_model
        integral_error
    end
end
        