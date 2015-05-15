classdef px4estimator < handle
    %   ESTIMATOR Attitude estimator for quadrotor
    %   Implements EKF for estimating the body frame velocities, pitch,
    %   roll, yaw rate, and down position for a quadrotor.
    %   Based on existing firmware for PX4
    
    properties
        
        % filter state
        xhat % state estimate
        P % covariance estimate
        
        % parameters
        S % input covariance
        Q % process covariance (hand tune)
        
        var_accel % variance for accelerometers
        var_gyro  % variance for gyros
        var_mag   %variance for magnetometer
    end
    
    methods
        
        % constructor
        function obj = estimator(std_dev_accel, std_dev_gyro, std_dev_mag, x0)
            
            obj.xhat = zeros(12,1);
            if (nargin > 3)
                if (size(x0) == size(obj.xhat))
                    obj.xhat = x0;
                else
                    disp(['estimator: expected x0 to be of size ' num2str(size(obj.xhat))...
                        ', got matrix of size ' num2str(size(x0)) '. Initalizing to zeros.'])
                end
            end

            obj.P = diag([.0001, .0001, .0001, .0001*(2*pi/190)^2, .0001*(2*pi/180)^2, .0001*(1*pi/180)^2, 0.0001,0.0001,0.0001);
            
            obj.var_accel = std_dev_accel^2;
            obj.var_gyro = std_dev_gyro^2;
            obj.var_mag = std_dev_mag^2;
            
            obj.S = diag([obj.var_gyro, obj.var_gyro, obj.var_gyro, obj.var_accel]);
            
            obj.Q = diag([.01, .01, .01, 0.01*(2*pi/180)^2, 0.01*(2*pi/180)^2, 0.01*(2*pi/180)^2, .01, .01, .01]);
        end
        
        % Runs the estimator main loop. This function propagates the model
        % and performs updates with any measurements that are available.
        % This function should be called each time a new IMU update is
        % available.
        function xhat = run_estimator(obj, accel, gyro, dt)
            
            inputs = [gyro; accel(3)];
            
            % propagation step
            for i = 1:obj.n
                % state
                obj.xhat = obj.xhat + (dt/obj.n) * obj.f(inputs);
                
                % covariance
                F_i = obj.F(inputs);
                M_i = obj.M(inputs);
                obj.P = obj.P + (dt/obj.n) * (F_i*obj.P + obj.P*F_i' + M_i*obj.S*M_i' + obj.Q);
%                 obj.P = obj.P + (dt/obj.n) * ( F_i * obj.P * F_i' + M_i*obj.S*M_i' + obj.Q );
            end
            
            % check for measurements
            y = [accel(1); accel(2)];
            
            if (obj.have_altimeter)
                y = [y; obj.altimeter];
            end
            
            if (obj.have_flow)
                y = [y; obj.flow];
            end
            
            y = [y; 0]; % constraint equation measurement
            
            % update step
            [h, H, N, R] = obj.update_matrices(inputs);
            K = obj.P * H' / (H*obj.P*H' + N*obj.S*N' + R);
            obj.xhat = obj.xhat + K*(y - h);
            obj.P = (eye(length(obj.xhat)) - K*H) * obj.P;
            
            % record that measurements have been used
            obj.have_altimeter = false;
            obj.have_flow = false;
            
            xhat = obj.xhat;
        end
        
        % Sets the value of the altimeter and the altimeter reading flag.
        % Should be  called each time a new altimeter reading is available.
        function set_altimeter(obj, altimeter)
            obj.altimeter = altimeter;
            obj.have_altimeter = true;
        end
        
        % Sets the value of the flow reading and the flow reading flag.
        % Should be called each time a new flow reading is available.
        function set_flow(obj, flow)
            obj.flow = flow;
            obj.have_flow = true;
        end
    end
    
    methods (Access = protected)
    
        function f = f(obj, inputs)
            
            u     = obj.xhat(1);
            v     = obj.xhat(2);
            w     = obj.xhat(3);
            phi   = obj.xhat(4);
            theta = obj.xhat(5);
            
            p    = inputs(1);
            q    = inputs(2);
            r    = inputs(3);
            am_k = inputs(4);
            
            f = [
                -obj.g*sin(theta) + (v*r - w*q) - obj.mu/obj.m*u;
                obj.g*sin(phi)*cos(theta) + (w*p - u*r) - obj.mu/obj.m*v;
                obj.g*cos(phi)*cos(theta) + am_k;
                p + (q*sin(phi) + r*cos(phi))*tan(theta);
                q*cos(phi) - r*sin(phi);
                0;
                -u*sin(theta) + v*sin(phi)*cos(theta) + w*cos(phi)*cos(theta)
                ];
        end
        
        function F = F(obj, inputs)
            
            u      = obj.xhat(1);
            v      = obj.xhat(2);
            w      = obj.xhat(3);
            phi    = obj.xhat(4);
            theta  = obj.xhat(5);
            
            p    = inputs(1);
            q    = inputs(2);
            r    = inputs(3);
            
            sphi = sin(phi);
            cphi = cos(phi);
            stheta = sin(theta);
            ctheta = cos(theta);
            
            F = [
                -obj.mu/obj.m   r               -q          0                           -obj.g*ctheta                       0   0
                -r              -obj.mu/obj.m   p           obj.g*cphi*ctheta           -obj.g*sphi*stheta                  0   0
                0               0               0           -obj.g*sphi*ctheta          -obj.g*cphi*stheta                  0   0
                0               0               0           (q*cphi-r*sphi)*tan(theta)  (q*sphi+r*cphi)/ctheta^2            0   0
                0               0               0           -q*sphi-r*cphi              0                                   0   0
                0               0               0           0                           0                                   0   0
                -stheta         sphi*ctheta     cphi*ctheta v*cphi*ctheta-w*sphi*ctheta -u*ctheta-(v*sphi-w*cphi)*stheta    0   0
                ];
        end
        
        function M = M(obj, ~)
            
            u      = obj.xhat(1);
            v      = obj.xhat(2);
            w      = obj.xhat(3);
            phi    = obj.xhat(4);
            theta  = obj.xhat(5);
            
            sphi = sin(phi);
            cphi = cos(phi);
            ttheta = tan(theta);
            
            M = [
                0       -w              v               0
                w       0               -u              0
                0       0               0               1
                1       sphi*ttheta     cphi*ttheta     0
                0       cphi            -sphi           0
                0       0               0               0
                0       0               0               0
                ];
        end
        
        function [h, H, N, R] = update_matrices(obj, inputs)
            
            u      = obj.xhat(1);
            v      = obj.xhat(2);
            w      = obj.xhat(3);
            phi    = obj.xhat(4);
            theta  = obj.xhat(5);
            psidot = obj.xhat(6);
            z      = obj.xhat(7);
            
            p    = inputs(1);
            q    = inputs(2);
            r    = inputs(3);
            
            sphi = sin(phi);
            cphi = cos(phi);
            ctheta = cos(theta);
            
            % IMU measurements (always have)
            h = [
                (v*r - w*q) - obj.mu/obj.m*u
                (w*p - u*r) - obj.mu/obj.m*v
                ];
            H = [
                -obj.mu/obj.m   r               -q      0       0       0       0
                -r              -obj.mu/obj.m   p       0       0       0       0
                ];
            N = [
                0    -w    v    0
                w    0     -u   0
                ];
            R_values = [obj.var_accel, obj.var_accel];
            
            % check for altimeter measurement
            if (obj.have_altimeter)
                h = [h;
                    -z
                    ];
                H = [H;
                    0   0   0   0   0   0   -1
                    ];
                N = [N;
                    0   0   0   0
                    ];
                R_values = [R_values, obj.var_alt];
            end
            
            % check for flow measurement
            if (obj.have_flow)
                h = [h;
                    u
                    v
                    ];
                H = [H;
                    1   0   0   0   0   0   0
                    0   1   0   0   0   0   0
                    ];
                N = [N;
                    0   0   0   0
                    0   0   0   0
                    ];
                R_values = [R_values, obj.var_flow, obj.var_flow];
            end
            
            % yaw rate constraint equation (always have)
            h = [h;
                psidot - (q*sin(phi) + r*cos(phi))/cos(theta)
                ];
            H = [H;
                0   0   0   -(q*cphi-r*sphi)/ctheta     -(q*sphi+r*cphi)*tan(theta)/ctheta  1   0
                ];
            N = [N;
                0   -sphi/ctheta    -cphi/ctheta    0
                ];
            R_values = [R_values, 0];
            
            R = diag(R_values);
        end
    end
end

