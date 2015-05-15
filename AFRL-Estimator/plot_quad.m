function plot_quad(time,tmp,position,vert_vel,thrust)
%
    figure(1)
    plot(time,tmp(1,:)*180/pi,'r');
    hold on
    plot(time,tmp(2,:)*180/pi,'g');
    plot(time,tmp(3,:)*180/pi,'b');
    title('Attitude Adjustment');
    xlabel('Time (s)');
    ylabel('Angle (Deg)');
    legend('Roll','Pitch','Yaw');
%}

%
    figure(2)
    plot3(position(1,1:length(time)/2),position(2,1:length(time)/2),position(3,1:length(time)/2));
    hold on
    plot3(position(1,length(time)/2:length(time)),position(2,length(time)/2:length(time)),position(3,length(time)/2:length(time)),'r');
    axis equal
    title('Position');
    xlabel('X Position');
    ylabel('Y Position');
    zlabel('Altitude');
    legend('First half of simulation','Second half of simulation');
%}

%{
    figure(3)
    plot(time,vert_vel,'k');
    title('Velocity Along Body Z Axis');
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
%}

%{
    figure(4)
    plot(time,thrust(1,:),'r');
    hold on
    plot(time,thrust(2,:),'g');
    plot(time,thrust(3,:),'b');
    plot(time,thrust(4,:),'k');
    title('Thrust');
    xlabel('Time (s)');
    ylabel('Thrust (N)');
    legend('Motor 1','Motor 2','Motor 3','Motor 4');
%}
end