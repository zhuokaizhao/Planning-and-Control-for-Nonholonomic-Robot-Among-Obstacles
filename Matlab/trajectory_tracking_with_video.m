% using feedback linearization method to do trajectory tracking
% the system is
%   dx1 = u1cos(theta)
%   dx2 = u1sin(theta)
%   dtheta = u2
%   y = [x1, x2]'
% control law is
%   v = ddy = ddyd - kp(y-yd) - kd(dy-ddyd)
% data loaded t, x1, x2, dx1, dx2, ddx1, ddx2, theta 

clear;clc;

% load trajectory
load('/final_trajectory.mat');
% plot planned trajectory
desired_path = [final_trajectory(:,2) final_trajectory(:, 3)];
% control gains
Kp = 3;
Kd = 2;
% number of small time interval in each big time step
N = 10;
% initial stale
y = [0;0];
dy = [0;0];
theta = 0;
Yt = y;
u1 = 1;
u2 = 0;
% car body dimensions
body_length = 40;
body_width = 24;
rear_wheel_position_ratio = 0.35;
[num_rows, num_colums] = size(final_trajectory);
i = 1;
% trajectory tracking

% obtain all the obstacles (rectangles)
obstacles = [66.5 187.5 87 143; 202.5 83.5 113 52; 264.5 237.5 114 27; 320.5 366.5 114 51];
num_obstacles = 4;

% initilize a video writer object
VW = VideoWriter('trajectory_tracking_simulation.avi', 'Uncompressed AVI');
VW.FrameRate = 100;
open(VW);
f = figure;
counter = 0;
while(i<num_rows)
    current_state = final_trajectory(i,:);
    delta_t = final_trajectory(i+1,1) - final_trajectory(i,1);
    % get desired states
    yd = [current_state(2); current_state(3)];
    dyd = [current_state(4); current_state(5)];
    ddyd = [current_state(6); current_state(7)];
    % update system
    for(j=1:1:N) 
        counter = counter + 1;
        set(0,'currentfigure',f); % set f the current figure
        clf; 
        v = ddyd - Kp*(y-yd)-Kd*(dy-dyd);
        RT = rotation_inv(theta);
        virtual_input = RT*v;
        % in case u1 is zero
        if(u1>0 && u1<10)
            dummy_u1 = 10;
        elseif(u1<0 && u1>-10)
            dummy_u1 = -10;
        else
            dummy_u1 = u1;
        end
        u2 = virtual_input(2)/dummy_u1;
        u1 = u1+virtual_input(1)*(delta_t/N);
        dy = [u1*cos(theta); u1*sin(theta)];
        y = y + [u1*cos(theta); u1*sin(theta)]*(delta_t/N);
        theta = theta+u2*(delta_t/N);
        theta = wrapToPi(theta);
        Yt = [Yt y];
        axis([0, 500, 0, 500]);
        hold on
        for jj = 1:num_obstacles
            rectangle('Position', obstacles(jj, :), 'FaceColor', [0 .5 .5]);
        end
        plot(desired_path(:,1), desired_path(:,2), 'r');
        [FP_x, FP_y] = getFourPoints(y(1), y(2), theta, rear_wheel_position_ratio, body_length, body_width);
        plot(FP_x, FP_y, 'b');
        plot([FP_x(4),FP_x(1)],[FP_y(4), FP_y(1)], 'b');
        plot(Yt(1,:), Yt(2,:), 'b');
        legend('desired path', 'tracked path', 'Location', 'NW');
        %axis equal;
        set(f,'color','w');
        current_frame = getframe(f);
        writeVideo(VW, current_frame);
        pause(0.01);
    end
    i = i+1;
end
close(VW);

function RT = rotation_inv(theta)
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    RT = R';
end

function [X, Y] = getFourPoints(center_x, center_y, theta, r, l, b)
    x0 = [center_x; center_y];
    R = [cos(theta) -sin(theta);
        sin(theta) cos(theta)];
    p1 = [-r*l;-0.5*b];
    p2 = [-r*l;0.5*b];
    p3 = [r*l;0.5*b];
    p4 = [r*l;-0.5*b];
    p1 = R*p1 + x0;
    p2 = R*p2 + x0;
    p3 = R*p3 + x0;
    p4 = R*p4 + x0;
    X = [p1(1),p2(1),p3(1),p4(1)];
    Y = [p1(2),p2(2),p3(2),p4(2)];
end