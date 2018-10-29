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
plot(desired_path(:,1), desired_path(:,2), 'r');
hold on
% control gains
Kp = 1;
Kd = 1;
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
u1_t = [];
u2_t = [];
all_t = [];
global_t = 0;
while(i<num_rows)
    current_state = final_trajectory(i,:);
    delta_t = final_trajectory(i+1,1) - final_trajectory(i,1);
    % get desired states
    yd = [current_state(2); current_state(3)];
    dyd = [current_state(4); current_state(5)];
    ddyd = [current_state(6); current_state(7)];
    % update system
    for(j=1:1:N) 
        v = ddyd - Kp*(y-yd)-Kd*(dy-dyd);
        RT = rotation_inv(theta);
        virtual_input = RT*v;
        % incase u1 is zero
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
        u1_t = [u1_t u1/100];
        u2_t = [u2_t u2];
        global_t = global_t + delta_t;
        all_t = [all_t global_t];
    end
    i = i+1;
end
plot(Yt(1,:), Yt(2,:), 'b');
legend('desired path', 'trajected path', 'Location', 'NW');
hold off;
figure()
plot(all_t, u1_t,'b');
legend('linear velocity input', 'Location', 'NW');
xlabel('time');
ylabel('linear velocity m/s');
figure()
plot(all_t, u2_t, 'b');
legend('angular velocity input', 'Location', 'NW');
xlabel('time');
ylabel('angular velocity rad/s');

function RT = rotation_inv(theta)
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    RT = R';
end