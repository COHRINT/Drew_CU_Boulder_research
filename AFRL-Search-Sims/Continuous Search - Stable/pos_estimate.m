classdef pos_estimate < handle
    properties
        state %x,y,u,v,heading,udot,vdot
        P
        Q
        R
    end
    methods
        function obj = pos_estimate(initial_state)
            obj.state = initial_state;
            obj.P = diag([0.05,.05,0.0001,0,0,0,0]);
            obj.Q = diag([.01,.01,.01,.01,0.01,.1,.1]);
            obj.R = diag([.01,.01,.01,.001,0.01,0.1,0.1]);
        end    
        
        function run_ekf(obj,imu,gps)
            %Build f matrix
            global dt;
            u = obj.state(3);
            v = obj.state(4);
            psi = obj.state(5);
            udot = obj.state(6);
            vdot = obj.state(7);
            Q_temp = obj.Q;
            R_temp = obj.R;
            P_t = obj.P;
            %=================PROPOGATE===================================
            x_t = obj.state + [u*cos(psi)-v*sin(psi);-u*sin(psi)-v*cos(psi);udot;vdot;0;0;0]*dt;
            
            F = eye(size(obj.state));
            F(1,3) = cos(psi)*dt;
            F(1,4) = -sin(psi)*dt;
            F(1,5) = dt*(-u*sin(psi)-v*cos(psi));
            F(2,3) = -sin(psi)*dt;
            F(2,4) = -cos(psi)*dt;
            F(2,5) = dt*(-u*cos(psi)+v*sin(psi));
            F(3,6) = dt;
            F(4,7) = dt;
            
            P_t = F*P_t*F' + Q_temp*dt;
            
            %====================UPDATE===============================
            if isempty(gps)
                z_k = [x_t(1:5)',imu(1:2)]';
            else
                z_k = [gps(1:4),x_t(5),imu(1:2)]';
            end
            
            h = [x_t(1:5)',x_t(6:7)']';
            
            y = z_k -h;
            
            H = eye(size(z_k,1));
            
            S = H*P_t*H' + R_temp*dt;

            K_t = P_t*H'/S;
            x_t = x_t + K_t*y;
            if ~isempty(gps)
                P_t = (eye(size(P_t)) - K_t*H)*P_t;
            end
            obj.state = x_t;
            obj.P = P_t;
            
        end
    end
end