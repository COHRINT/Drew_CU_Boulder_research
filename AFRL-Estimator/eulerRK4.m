classdef eulerRK4
    methods
        function obj = eulerRK4(State, Ib, m, Force, Moment);
            obj.State = State;
            obj.Ib = Ib;
            obj.m = m;
            obj.Force = Force;
            obj.Moment = Moment;
        end
        function dState = stateDiff(obj, y)
            
            Po = y(1:3);
            Or = y(4:6);
            Ve = y(7:9);
            An = y(10:12);
            
            %{
            gravity = [cos(Or(1))*cos(Or(2)) cos(Or(1))*sin(Or(2))*sin(Or(3))-sin(Or(1))*cos(Or(3)) cos(Or(1))*sin(Or(2))*cos(Or(3))+sin(Or(1))*sin(Or(3));...
                       sin(Or(1))*cos(Or(2)) sin(Or(1))*sin(Or(2))*sin(Or(3))+cos(Or(1))*cos(Or(3)) sin(Or(1))*sin(Or(2))*cos(Or(3))-cos(Or(1))*sin(Or(3));...
                       -sin(Or(2)) cos(Or(2))*sin(Or(3)) cos(Or(2))*cos(Or(3))]*[0;0;obj.m*32.2];
            %}

            gravity = transpose([cos(Or(2))*cos(Or(3)) -cos(Or(2))*sin(Or(3)) sin(Or(2));...
                       cos(Or(1))*sin(Or(3))+cos(Or(3))*sin(Or(1))*sin(Or(2)) cos(Or(1))*cos(Or(3))-sin(Or(1))*sin(Or(2))*sin(Or(3)) -cos(Or(2))*sin(Or(1));...
                       sin(Or(1))*sin(Or(3))-cos(Or(1))*cos(Or(3))*sin(Or(2)) cos(Or(3))*sin(Or(1))+cos(Or(1))*sin(Or(2))*sin(Or(3)) cos(Or(1))*cos(Or(2))])*[0;0;obj.m*9.81];
            % Define the three matrices used in calculating the diff eqs
            TransKinDiffEq = [cos(Or(2))*cos(Or(3)) -cos(Or(2))*sin(Or(3)) sin(Or(2));...
                       cos(Or(1))*sin(Or(3))+cos(Or(3))*sin(Or(1))*sin(Or(2)) cos(Or(1))*cos(Or(3))-sin(Or(1))*sin(Or(2))*sin(Or(3)) -cos(Or(2))*sin(Or(1));...
                       sin(Or(1))*sin(Or(3))-cos(Or(1))*cos(Or(3))*sin(Or(2)) cos(Or(3))*sin(Or(1))+cos(Or(1))*sin(Or(2))*sin(Or(3)) cos(Or(1))*cos(Or(2))];
                   
            RotKinDiffEq = [1, sin(Or(1))*tan(Or(2)), cos(Or(1))*tan(Or(2)); 0, cos(Or(1)), -sin(Or(1)); 0, sin(Or(1))/cos(Or(2)), cos(Or(1))/cos(Or(2))];

            DyDiffEq = [0, -An(3), An(2);An(3),0, -An(1);-An(2), An(1), 0];

            % Calculate Differential Equations
            dPo = TransKinDiffEq*Ve;
            dOr = RotKinDiffEq'*An;
            dVe = (1/obj.m)*(obj.Force+gravity) - DyDiffEq*Ve;
            tmp = obj.Moment - DyDiffEq*obj.Ib*An;
            dAn = obj.Ib\tmp;

            dState(1:3,1) = dPo;
            dState(4:6,1) = dOr;
            dState(7:9,1) = dVe;
            dState(10:12,1) = dAn;
        end
        function newState = homebrewRK4(obj)
            dt = 0.01;
            k1 = obj.stateDiff(obj.State);
            k2 = obj.stateDiff(obj.State+(dt/2)*k1);
            k3 = obj.stateDiff(obj.State+(dt/2)*k2);
            k4 = obj.stateDiff(obj.State+dt*k3);
            newState = obj.State + (dt/6)*(k1+2*k2+2*k3+k4);
        end
        
    end
    
    properties
            State
            Ib
            m
            Force
            Moment
    end
 end
    