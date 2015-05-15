classdef agent_truth < handle
    properties
        x_pos
        y_pos
        u
        v
        udot
        vdot
        heading
        radius_of_detection
        angle_of_detection
        commanded_pos
        alpha
        beta
    end
    methods
        function obj = agent_truth(x,y,u,v,heading,radius_of_detection,angle_of_detection,alpha,beta)
            if nargin ~= 9
                disp(['agent: expected [' num2str(9)...
                        '] arguments, got [' num2str(nargin) ']. Initializing to default.'])
                    obj.x_pos = 0;
                    obj.y_pos = 0;
                    obj.u = 0;
                    obj.v = 0;
                    obj.udot = 0;
                    obj.vdot = 0;
                    obj.heading = 0;
                    obj.radius_of_detection = 1;
                    obj.angle_of_detection = 2*pi;
                    obj.alpha = .1;
                    obj.beta = .1;
            else
                obj.x_pos = x;
                obj.y_pos = y;
                obj.u = u;
                obj.v = v;
                obj.udot = 0;
                obj.vdot = 0;
                obj.heading = heading;
                obj.radius_of_detection = radius_of_detection;
                obj.angle_of_detection = angle_of_detection;
                obj.alpha = alpha;
                obj.beta = beta;
            end
        end    
    end
end