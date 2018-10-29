%% 530.678 Final Project - Path Planning Section
%% C-RRT* path-planning algorithm in 2D with obstacle avoidance

%% Zhuokai Zhao, Changxin Yan, Mengdi Xu
clear
clearvars
close all
disp('Program Starts');
disp('530.678 Final Project - Path Planning Section');
disp('Customized RRT* path-planning algorithm in 2D with obstacle avoidance');
disp('Zhuokai Zhao, Changxin Yan, Mengdi Xu');

%% Legends in graph
%% Blue crosses are the ramdomly picked node q_rand
%% Red crosses are q_new, which are generated when q_rand is too far away from q_near
%% Blue lines are qualified path between q_near and q_new
%% Red lines are after updating parent (finding q_nearest), which are between q_nearest and q_new
%% Yellow lines are showing the tree rewire 
%% Thick blue line is the line-connected final path
%% Thick green line is the final planning path after polynomial fit

% The input image is 500*500, converted to a 5m*5m real-world ground
% Each pixel means 1 cm

% Start and end position (x, y, theta)
q_start.coord = [0 0 0];
q_goal.coord = [499 499 0];

% Some control inputs
max_stepsize = 5;
numNodes = 2000;  
neigbor_radius = 50;
section_size = 50;
poly_degree = 4;

% used in obstacle cost function
rho = 40;
sigma = 30;

% If we want bias when generating new state q_rand
wantBias = 1;
bias_percentage = 0.5;

% used in polynomial fit
max_difference = 0.8;

% used in velocity assignment
% acceleration is in cm/s^2
a_max = 20;
% velocity is in cm/s
v_max = 20;

% if we want to plot velocities
wantVelocityPlot = true;

%% Part I: Import map image and generate the map
% Import our map image
filepath = '\Sample_Map.png';
image = imread(filepath);
%imshow(image)

% resize the image to be 500*500
if (length(image) ~= 500)
    image = imresize(image, [500 500]);
    disp('Input image has been resized to 500x500');
end

% convert the image to grayscale and then black and white
gray_image = rgb2gray(image);
%imshow(gray_image)
bw_image = gray_image < 240;
%figure();
%imshow(bw_image)

% get rectangles (obstacles) from the image
% find both black and white regions
stats = [regionprops(bw_image); regionprops(not(bw_image))];

% obtain all the obstacles (rectangles)
num_obstacles = numel(stats);
obstacles = [];

% plot detected rectangles (for report purposes)
%{
for i = 1:numel(stats)
    if stats(i).BoundingBox(3) < 400
        rectangle('Position', stats(i).BoundingBox, 'Linewidth', 3, 'EdgeColor', 'r', 'LineStyle', '--');
    end
end
%}

% count for disqualified obstacle
count = 0;
for i = 1:num_obstacles
    if stats(i).BoundingBox(3) > 400
        count = count + 1;
        continue;
    end
    obstacles = [obstacles; stats(i).BoundingBox];
end
num_obstacles = num_obstacles - count;

% get the size of the map and define some variables
[x_max, y_max] = size(bw_image);      

% define starting and goal configuration
q_start.cost = 0;
q_start.parent = 0;
q_goal.cost = 0;

% we restrict our initial theta to be something reasonable
q_start.coord(3) = atan2(q_goal.coord(2)-q_start.coord(2), q_goal.coord(1)-q_start.coord(1));

% q_goal, we let the goal have the same theta as the start
q_goal.coord(3) = q_start.coord(3);

% put q_start as the first visited nodes
num_visited = 0;
num_visited = num_visited + 1;
q_start.index = num_visited;
visited_nodes(q_start.index) = q_start;

% plot the map with obstalces
figure(1)
axis([0 x_max 0 y_max])
%axis equal

% draw the map with obstacles
for i = 1:num_obstacles
    rectangle('Position', obstacles(i, :), 'FaceColor', [0 .5 .5])
    hold on
end

tic
%% Path planning using RRT*
% q_prev is used when biasly randomizing q_rand
q_prev = q_start;
shouldStop = false;

for i = 1:numNodes
    % shows the progress percentage
    percentage_finished = 100*i/numNodes;
    clc
    fprintf('RRT* has completed: %4.2f %%\n', percentage_finished);
    
    % end the loop if the goal node can already be reached
    for j = 1:length(visited_nodes)
        if (mydist(visited_nodes(j), q_goal) <= max_stepsize)
            fprintf('Goal has been reached before randomly choosing all %d nodes.\n', numNodes);
            shouldStop = true;
        end
    end
    if (shouldStop)
        break
    end
    
    % now we have the randomized state
    q_rand.coord = [round(rand(1)*x_max) round(rand(1)*y_max) rand(1)*2*pi];
    q_rand.cost = 0;
    q_rand.index = 0;
    q_rand.parent = 0;
    if (wantBias == 1)
        [biased_x, biased_y, biased_theta] = getBiasedState(q_prev, q_goal, bias_percentage, x_max, y_max);
        q_rand.coord = [biased_x, biased_y, biased_theta];
    end
    plot(q_rand.coord(1), q_rand.coord(2), 'x', 'Color', 'blue');
    drawnow
    hold on
    
    % Pick q_near which is the closest node to q_rand
    min_dist = realmax;
    for j = 1:length(visited_nodes)
        cur_node = visited_nodes(j);
        cur_dist = mydist(cur_node, q_rand);
        if (cur_dist < min_dist)
            min_dist = cur_dist;
            q_near = cur_node;
        end
    end
    
    % we need q_new if q_rand is too far away from q_near
    % even if not too far away, we change name that makes q_new = q_rand
    q_new = get_q_new(q_near, q_rand, min_dist, max_stepsize);
    plot(q_new.coord(1), q_new.coord(2), 'x', 'Color', 'red')
    
    % Remember this current q_new for later use in the next loop
    q_prev.coord = q_new.coord;
    
    % if there is no collision between q_near and q_new, also enough space
    % around both q_near and q_new, connect the path
    collision = ifCollision(q_near, q_new, obstacles) || narrowSpace(q_near, obstacles) || narrowSpace(q_new, obstacles);
    if (collision == 1)
        %fprintf("WARNING: q_near(%d, %d) and q_new(%d, %d) could not be connected due to collision.\n", q_near.coord(1), q_near.coord(2), q_new.coord(1), q_new.coord(2));
    else
        %line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'blue');
        %drawnow
        %hold on
        
        % Within a radius of r, find all nearest candidates nodes of q_new
        q_neighbor = [];
        index = 1;
        for j = 1:length(visited_nodes)
            if ((ifCollision(visited_nodes(j), q_new, obstacles)==0) && mydist(visited_nodes(j), q_new) <= neigbor_radius)
                q_neighbor(index).coord = visited_nodes(j).coord;
                q_neighbor(index).cost = visited_nodes(j).cost;
                q_neighbor(index).parent = visited_nodes(j).parent;
                q_neighbor(index).index = visited_nodes(j).index;
                index = index + 1;
            end
        end
        
        % Initialize cost to currently known value
        q_min = q_near;
        % cost function contains the distance from from the previous step as well as the distance to obstacle
        C_min = q_near.cost + mydist(q_near, q_new) + obstacle_cost(obstacles, q_new, rho, sigma);
        
        % Iterate through all near neighbors to find alternate lower cost paths
        for k = 1:length(q_neighbor)
            if ((ifCollision(q_neighbor(k), q_new, obstacles)==0) && (q_neighbor(k).cost + mydist(q_neighbor(k), q_new) + obstacle_cost(obstacles, q_new, rho, sigma) < C_min))
                q_min = q_neighbor(k);
                C_min = q_neighbor(k).cost + mydist(q_neighbor(k), q_new) + obstacle_cost(obstacles, q_new, rho, sigma);
            end
        end
        
        % set the parent for q_new
        q_new.parent = q_min.index;
        % draw the line that has the closest distance to reach q_new
        line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'red', 'LineWidth', 1);                
        hold on
        %{
        for j = 1:length(visited_nodes)
            if visited_nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end
        %}
        
        % Rewire the tree - Check if q_new can be parent of any of the
        % q_nearest candidates
        for k = 1:length(q_neighbor)
            if ((ifCollision(q_new, q_neighbor(k), obstacles)==0) && (q_new.cost + mydist(q_new, q_neighbor(k)) + obstacle_cost(obstacles, q_neighbor(k), rho, sigma) < q_neighbor(k).cost))
                q_neighbor(k).parent = q_new.index;
                line([q_new.coord(1), q_neighbor(k).coord(1)], [q_min.coord(2), q_neighbor(k).coord(2)], 'Color', 'yellow', 'LineWidth', 1);                
                hold on
            end
        end
        
        % add q_new to visited nodes
        num_visited = num_visited + 1;
        q_new.index = num_visited;
        visited_nodes(num_visited) = q_new;
        
    end
end

%% Search backwards from goal to start to find the optimal least cost path
min_dist_to_goal = realmax;
last_node_index = 0;
for j = 1:length(visited_nodes)
    cur_dist_to_goal = mydist(visited_nodes(j), q_goal);
    if (cur_dist_to_goal < min_dist_to_goal)
        min_dist_to_goal = cur_dist_to_goal;
        last_node_index = j;
    end
end

q_final = visited_nodes(last_node_index);
q_goal.parent = q_final.index;
num_visited = num_visited + 1;
q_goal.index = num_visited;
visited_nodes(num_visited) = q_goal;

q_end = q_goal;

% check if the closest node to goal is good
if (ifCollision(q_final, q_goal, obstacles))
    fprintf('WARNING: q_final(%d, %d) and q_goal(%d, %d) could not be connected due to collision.\n', q_final.coord(1), q_final.coord(2), q_goal.coord(1), q_goal.coord(2));
end
if (mydist(q_final, q_goal) > max_stepsize)
    disp('WARNING: The distance between q_final and q_goal is larger than maximum stepsize.');
end

% generate the optimal path
optimal_path = [q_end.coord];

while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), visited_nodes(start).coord(1)], [q_end.coord(2), visited_nodes(start).coord(2)], 'Color', 'blue', 'LineWidth', 2);
    hold on
    cur_node = visited_nodes(start).coord;
    optimal_path = [cur_node; optimal_path];
    q_end = visited_nodes(start);
end

%% fit the polynomial
% optimal path contains all the states of the path
% divide the steps into different sections, then fit polynomials between
% them, so that the whole path is continuous
poly_path_x = [];
poly_path_y = [];
%poly_path_y = zeros(1, length(optimal_path));

% curvature of each polynomial segment, used for speed assignment
myCurvatures = [];

% determine different situations
if (rem(length(optimal_path), section_size) == 0) 
    num_section = length(optimal_path)/section_size;
    for i = 1:num_section
        if (i == 1)
            % if this is the first polynomial, we need to constraint the
            % first node
            cur_poly_path_x = optimal_path((i-1)*section_size+1:i*section_size, 1)';
            cur_y = optimal_path((i-1)*section_size+1:i*section_size, 2)';
            p = polyfix(cur_poly_path_x', cur_y', cur_poly_path_x(1, 1), cur_y(1, 1));
            cur_poly_path_y = polyval(p, cur_poly_path_x);
            
            % save the fitting result
            poly_path_x = [poly_path_x, cur_poly_path_x];
            poly_path_y = [poly_path_y, cur_poly_path_y];
            
            % save the current x, y and gradient for next round's use
            prev_poly_path_x = cur_poly_path_x;
            prev_poly_path_y = cur_poly_path_y;
            prev_poly_deri_y = gradient(prev_poly_path_y);
            
            % save the curvature information at the same time
            curCurvature = getCurvature(cur_poly_path_y);
            myCurvatures = [myCurvatures, curCurvature];
        elseif (i == num_section)
            % if this is the last one, we need to also restrict the last
            cur_poly_path_x = optimal_path((i-1)*section_size+1:i*section_size, 1)';
            cur_y = optimal_path((i-1)*section_size+1:i*section_size, 2)';
            p = polyfix(cur_poly_path_x', cur_y', poly_degree, [prev_poly_path_x(1, end), cur_poly_path_x(1, end)], [prev_poly_path_y(1, end), cur_poly_path_x(1, end)], prev_poly_path_x(1, end), prev_poly_deri_y(1, end));
            cur_poly_path_y = polyval(p, cur_poly_path_x);
            
            % save the fitting result
            poly_path_x = [poly_path_x, cur_poly_path_x];
            poly_path_y = [poly_path_y, cur_poly_path_y];
            
            % save the current x, y and gradient for next round's use
            prev_poly_path_x = cur_poly_path_x;
            prev_poly_path_y = cur_poly_path_y;
            prev_poly_deri_y = gradient(prev_poly_path_y);
            
            % save the curvature information at the same time
            curCurvature = getCurvature(cur_poly_path_y);
            myCurvatures = [myCurvatures, curCurvature];
            
        else
            % when it is not the first polynomial, nor the last
            % we want include the previous last point to fix the start
            % point and the start derivative
            cur_poly_path_x = optimal_path((i-1)*section_size+1:i*section_size, 1)';
            cur_y = optimal_path((i-1)*section_size+1:i*section_size, 2)';
            p = polyfix(cur_poly_path_x', cur_y', poly_degree, prev_poly_path_x(1, end), prev_poly_path_y(1, end), prev_poly_path_x(1, end), prev_poly_deri_y(1, end));
            cur_poly_path_y = polyval(p, cur_poly_path_x);
            
            % save the fitting result
            poly_path_x = [poly_path_x, cur_poly_path_x];
            poly_path_y = [poly_path_y, cur_poly_path_y];
            
            % save the current x, y and gradient for next round's use
            prev_poly_path_x = cur_poly_path_x;
            prev_poly_path_y = cur_poly_path_y;
            prev_poly_deri_y = gradient(prev_poly_path_y);
            
            % save the curvature information at the same time
            curCurvature = getCurvature(cur_poly_path_y);
            myCurvatures = [myCurvatures, curCurvature];
        end
    end
else
    num_section = floor(length(optimal_path)/section_size) + 1;
    for i = 1:num_section-1
        if (i == 1)
            % if this is the first polynomial, we need to constraint the
            % first node
            cur_poly_path_x = optimal_path((i-1)*section_size+1:i*section_size, 1)';
            cur_y = optimal_path((i-1)*section_size+1:i*section_size, 2)';
            p = polyfix(cur_poly_path_x', cur_y', poly_degree, cur_poly_path_x(1, 1), cur_y(1, 1));
            cur_poly_path_y = polyval(p, cur_poly_path_x);
            
            % save the fitting result
            poly_path_x = [poly_path_x, cur_poly_path_x];
            poly_path_y = [poly_path_y, cur_poly_path_y];
            
            % save the current x, y and gradient for next round's use
            prev_poly_path_x = cur_poly_path_x;
            prev_poly_path_y = cur_poly_path_y;
            prev_poly_deri_y = gradient(prev_poly_path_y);
            
            % save the curvature information at the same time
            curCurvature = getCurvature(cur_poly_path_y);
            myCurvatures = [myCurvatures, curCurvature];
        else
            % when it is not the first polynomial, we want to also include 
            % the previous last point, fix the start point and the start derivative
            cur_poly_path_x = optimal_path((i-1)*section_size+1:i*section_size, 1)';
            cur_y = optimal_path((i-1)*section_size+1:i*section_size, 2)';
            p = polyfix(cur_poly_path_x', cur_y', poly_degree, prev_poly_path_x(1, end), prev_poly_path_y(1, end), prev_poly_path_x(1, end), prev_poly_deri_y(1, end));
            cur_poly_path_y = polyval(p, cur_poly_path_x);
            
            % save the fitting result
            poly_path_x = [poly_path_x, cur_poly_path_x];
            poly_path_y = [poly_path_y, cur_poly_path_y];
            
            % save the current x, y and gradient for next round's use
            prev_poly_path_x = cur_poly_path_x;
            prev_poly_path_y = cur_poly_path_y;
            prev_poly_deri_y = gradient(prev_poly_path_y);
            
            % save the curvature information at the same time
            curCurvature = getCurvature(cur_poly_path_y);
            myCurvatures = [myCurvatures, curCurvature];
        
        end
    end
    % the residual states
    % when it is not the first polynomial, we want to also include the previous last point, fix the start
    % point and the start derivative
    cur_poly_path_x = optimal_path(i*section_size+1:length(optimal_path), 1)';
    cur_y = optimal_path(i*section_size+1:length(optimal_path), 2)';
    p = polyfix(cur_poly_path_x', cur_y', 2, [prev_poly_path_x(1, end), cur_poly_path_x(1, end)], [prev_poly_path_y(1, end), cur_poly_path_x(1, end)], prev_poly_path_x(1, end), prev_poly_deri_y(1, end));
    cur_poly_path_y = polyval(p, cur_poly_path_x);

    % save the current x, y and gradient for next round's use
    prev_poly_path_x = cur_poly_path_x;
    prev_poly_path_y = cur_poly_path_y;
    prev_poly_deri_y = gradient(prev_poly_path_y);

    % save the fitting result
    poly_path_x = [poly_path_x, cur_poly_path_x];
    poly_path_y = [poly_path_y, cur_poly_path_y];

    % save the curvature information at the same time
    curCurvature = getCurvature(cur_poly_path_y);
    myCurvatures = [myCurvatures, curCurvature];
end

% plot the polynomial
plot(poly_path_x(1, :), poly_path_y(1, :), 'Color', 'green', 'LineWidth', 2)

% add the theta information to get the final path with (x, y, theta)
% note that final_path's y is from polynomial fit
final_path = [];
for i = 1:length(poly_path_y)
    final_path = [final_path;
                  poly_path_x(1, i), poly_path_y(1, i)];
end

time_used = toc;
fprintf('RRT* has completed in %4.2f seconds\n', time_used);
%load handel
%sound(y,Fs)

%% Assign velocities and time stamp to each state, based on curvatures
velocities = zeros(1, length(myCurvatures));
for i = 1:length(myCurvatures)
    % v = min(sqrt(a_max/cur_k), v_max)
    velocities(1, i) = min(sqrt(a_max/myCurvatures(1,i)), v_max);
end

time = zeros(1, length(myCurvatures));
for i = 2:length(myCurvatures)
    % first compute the average v = (v_(i-1) + v_i)/2 
    avg_v = (velocities(1, i-1) + velocities(1, i))/2;
    del_l = sqrt((poly_path_x(1, i) - poly_path_x(1, i-1))^2 + (poly_path_y(1, i) - poly_path_y(1, i-1))^2);
    del_t = del_l/avg_v;
    time(1, i) = time(1, i-1) + del_t;
end

%% Generate the final trajectory with updated theta, i.e. [t, x, y, x_dot, y_dot, theta]
final_trajectory = [];
% compute the theta
all_theta = computeTheta(final_path);
for i = 1:length(poly_path_y)
    cur_x_dot = velocities(1, i) * cos(all_theta(1, i));
    cur_y_dot = velocities(1, i) * sin(all_theta(1, i));
    final_trajectory = [final_trajectory;
                        time(1, i), final_path(i, 1), final_path(i, 2), cur_x_dot, cur_y_dot, all_theta(1, i)];
end
% plot to see if the velocities are in the correct directions
% quiver(x, y, dx, dy)
if (wantVelocityPlot)
    quiver(final_trajectory(1:10:end, 2), final_trajectory(1:10:end, 3), final_trajectory(1:10:end, 4), final_trajectory(1:10:end, 5), 'color', 'm', 'LineWidth', 1);
end

% compute ddx and ddy (a_x and a_y) and add to final trajectory
ddx = gradient(final_trajectory(:, 4));
ddy = gradient(final_trajectory(:, 5));

final_trajectory(:, 6) = ddx;
final_trajectory(:, 7) = ddy;

% add theta to the final trajectory
final_trajectory(:, 8) = all_theta(1, :)';

save('final_trajectory_new.mat', 'final_trajectory');

%% Helper functions
% helper function that randomize theta with bias 
function [biased_x, biased_y, biased_theta] = getBiasedState(q_prev, q_goal, bias_percentage, x_max, y_max)
    % initial assignment (without bias)
    biased_x = round(rand(1)*x_max);
    biased_y = round(rand(1)*y_max);
    biased_theta = rand(1)*2*pi;
    
    % randomized q_rand will have a higher bias of getting the [x y theta] with more reasonable values
    if (q_prev.coord == q_goal.coord)
        fprintf('WARNING: q_prev = q_goal');
    end
    
    bias_percentage = round(bias_percentage*10);
    bias_box = zeros(1, 10);
    for i = 1:bias_percentage
        bias_box(1, i) = 1;
    end
    pick = bias_box(randi(10));

    % if x_goal < x_prev, biased to [x < x_prev], [pi/2, 3*pi/2]
    if (q_goal.coord(1) < q_prev.coord(1))
        % if y_goal < y_prev, [0 x_prev) [0 y_prev) [pi, 3*pi/2] are the biased range
        if (q_goal.coord(2) < q_prev.coord(2))
            if (pick == 1)
                biased_x = q_prev.coord(1) - round(rand(1)*q_prev.coord(1));
                biased_y = q_prev.coord(2) - round(rand(1)*q_prev.coord(2));
                biased_theta = rand(1)*pi/2 + pi;                   
            end
        end
        % if y_goal > y_start, [0 x_prev) (y_prev y_max] [pi/2, pi] is the higher biased range
        if (q_goal.coord(2) > q_prev.coord(2))
            if (pick == 1)
                biased_x = q_prev.coord(1) - round(rand(1)*q_prev.coord(1));
                biased_y = q_prev.coord(2) + round(rand(1)*(y_max - q_prev.coord(2)));
                biased_theta = rand(1)*pi/2 + pi/2;
            end
        end
    end

    % if x_goal > x_start, biased to [x > x_prev], [0, pi/2] & [3*pi/2, 2*pi]
    if (q_goal.coord(1) > q_prev.coord(1))
        % if y_goal < y_start, (x_prev x_max], [0 y_prev] [3*pi/2, 2*pi] are the biased range
        if (q_goal.coord(2) < q_prev.coord(2))
            if (pick == 1)
                biased_x = q_prev.coord(1) + round(rand(1)*(x_max - q_prev.coord(1)));
                biased_y = q_prev.coord(2) - round(rand(1)*q_prev.coord(2));
                biased_theta = rand(1)*pi/2 + 3*pi/2;                   
            end
        end
        % if y_goal > y_start, (x_prev x_max], (y_prev y_max], [0, pi/2] are the biased range
        if (q_goal.coord(2) > q_prev.coord(2))
            if (pick == 1)
                biased_x = q_prev.coord(1) + round(rand(1)*(x_max - q_prev.coord(1)));
                biased_y = q_prev.coord(2) + round(rand(1)*(y_max - q_prev.coord(2)));
                biased_theta = rand(1)*pi/2;
            end
        end
    end
end
% Get the distance between two nodes
function d = mydist(q1, q2)

% distance without theta
%d = sqrt((q1.coord(1)-q2.coord(1))^2 + (q1.coord(2)-q2.coord(2))^2);

% distance with theta
d = sqrt((q1.coord(1)-q2.coord(1))^2 + (q1.coord(2)-q2.coord(2))^2 + (q1.coord(3)-q2.coord(3))^2);

end

% return q_new if q_rand is too far away from q_near
function q_new = get_q_new(q_near, q_rand, dist, max_stepsize)
   q_new = q_rand;
   
   % Steer towards q_rand within maximum step size
   if dist >= max_stepsize
       q_new.coord(1) = q_near.coord(1) + ((q_rand.coord(1)-q_near.coord(1)) * max_stepsize) / mydist(q_near, q_rand);
       q_new.coord(2) = q_near.coord(2) + ((q_rand.coord(2)-q_near.coord(2)) * max_stepsize) / mydist(q_near, q_rand);
       q_new.coord(3) = q_near.coord(3) + ((q_rand.coord(3)-q_near.coord(3)) * max_stepsize) / mydist(q_near, q_rand);
   end   
   
end

% part of the cost function that takes the obstacles and exploring state as
% input, return the cost value
function cost = obstacle_cost(obstacles, q_new, rho, sigma)
    min_dist = realmax;
    % first we determine the nearest obstacle
    [num, ~] = size(obstacles);
    for i = 1:num
        % use the center of the obstacle to compute distance
        cur_center = [(obstacles(i, 1)+obstacles(i, 3))/2, (obstacles(i, 2)+obstacles(i, 3))/2];
        % we should take off the distance from sides to the center
        side_dist = sqrt(obstacles(i, 3)^2 + obstacles(i, 4)^2);
        cur_dist = sqrt((q_new.coord(1)-cur_center(1))^2 + (q_new.coord(2)-cur_center(2)^2)) - side_dist;
        if (cur_dist < min_dist)
            min_dist = cur_dist;
        end
    end
    
    % use the min distance in the cost computation: 
    % cost = rho * exp(-sigma * min_dist^2)
    cost = rho * exp(-sigma * min_dist^2);

end

% checks if two lines, q1q2 and e1e2 have crossing, note that this is
% irrelavent to the value of theta
function val = checkLineCross(q1, q2, e1, e2)

    x=[q1(1) q2(1) e1(1) e2(1)];
    y=[q1(2) q2(2) e1(2) e2(2)];
    dt1 = det([1,1,1;x(1),x(2),x(3);y(1),y(2),y(3)])*det([1,1,1;x(1),x(2),x(4);y(1),y(2),y(4)]);
    dt2 = det([1,1,1;x(1),x(3),x(4);y(1),y(3),y(4)])*det([1,1,1;x(2),x(3),x(4);y(2),y(3),y(4)]);

    if(dt1<=0 && dt2<=0)
        val = 1;         %If lines intesect
    else
        val = 0;
    end

end

% Checks if any collision happens if q1 and q2 are connected
function result = ifCollision(q1, q2, all_obstacles)
    % get the total number of obstacles that we need to worry about
    [num_obstacles, ~] = size(all_obstacles);
    % by default no collision
    result = 0;
    for i = 1:num_obstacles
        obstacle = all_obstacles(i, :);
        small_x = obstacle(1);
        large_x = obstacle(1) + obstacle(3);
        small_y = obstacle(2);
        large_y = obstacle(2) + obstacle(4);

        % left edge
        leftEdge_top = [small_x, large_y];
        leftEdge_bot = [small_x, small_y];
        
        % right edge
        rightEdge_top = [large_x, large_y];
        rightEdge_bot = [large_x, small_y];
        
        % top edge
        topEdge_left = [small_x, large_y];
        topEdge_right = [large_x, large_y];
        
        % bottom edge
        bottomEdge_left = [small_x, small_y];
        bottomEdge_right = [large_x , small_y];

        % Check if path from q1 to q2 intersects any of the four edges of the obstacle
        % left edge
        collision_left = checkLineCross(q1.coord, q2.coord, leftEdge_bot, leftEdge_top);
        % right edge
        collision_right = checkLineCross(q1.coord, q2.coord, rightEdge_bot, rightEdge_top);
        % top edge
        collision_top = checkLineCross(q1.coord, q2.coord, topEdge_left, topEdge_right);
        % bottom edge
        collision_bot = checkLineCross(q1.coord, q2.coord, bottomEdge_left,bottomEdge_right);
        
        
        if (collision_left || collision_right || collision_top || collision_bot)
            result = 1;
            break;
        end
    end
        
end

% The function checks if there is not enough space around q_near or q_new
function tooNarrow = narrowSpace(q, all_obstacles)
    % assume the eduMIP to be a disk with radius 40cm
    % define q_up to be at 40cm upper than q
    q_up = q;
    q_up.coord(2) = q.coord(2)+40;
    
    % define q_down to be at 40cm upper than q
    q_down = q;
    q_down.coord(2) = q.coord(2)-40;
    
    % define q_left to be at 40cm upper than q
    q_left = q;
    q_left.coord(1) = q.coord(1)-40;
    
    % define q_right to be at 40cm upper than q
    q_right = q;
    q_right.coord(1) = q.coord(1)+40;

    % check if line between q_temp and q will cause collision
    tooNarrow = ifCollision(q, q_up, all_obstacles) || ifCollision(q, q_down, all_obstacles) || ifCollision(q, q_left, all_obstacles) || ifCollision(q, q_right, all_obstacles);
    
end

% The function computes the curvature of a continuous polynomials
function k = getCurvature(f)
    % the formula of computing curvature is k = f''(x) / (1 + f'(x)^2)^(3/2)
    % source: http://tutorial.math.lamar.edu/Classes/CalcIII/Curvature.aspx
    df = gradient(f);
    ddf = gradient(df);
    k = zeros(1, length(f));
    for i = 1:length(f)
        k(1,i) = abs(ddf(i) / sqrt((1 + df(i)^2)^3));
    end
end

% Helper function that computes the correct theta
function all_theta = computeTheta(final_path)
    all_x = final_path(:, 1)';
    all_y = final_path(:, 2)';
    x_dot = gradient(all_x);
    y_dot = gradient(all_y);
    all_theta = atan2(y_dot, x_dot);
    for i = 1:length(all_theta)
        if (all_theta(i) < -pi/2)
            all_theta(i) = all_theta(i)+pi;
        end
    end
end

% Fit polynomial p to data, but specify value at specific points
function p = polyfix(x,y,n,xfix,yfix,xder,dydx)
    %% Make sure all input arrays are column vectors of compatible sizes:
    x = x(:);
    y = y(:);
    nfit = length(x);
    if ~(length(y)== nfit)
        error('x and y must have the same size');
    end
    xfix = xfix(:);
    yfix = yfix(:);
    nfix = length(xfix);
    if ~(length(yfix)== nfix)
        error('xfit and yfit must have the same size');
    end
    if nargin > 5 % Derivatives specified
        xder = xder(:);
        dydx = dydx(:);
    else
        xder = [];
        dydx = [];
    end
    nder = length(xder); 
    if ~(length(dydx) == nder)
        error('xder and dydx must have the same size');
    end
    nspec = nfix + nder;
    specval = [yfix;dydx];
 
    %% First find A and pc such that A*pc = specval
    A = zeros(nspec,n+1); 
    % Specified y values
    for i = 1:n+1
        A(1:nfix,i) = ones(nfix,1).*xfix.^(n+1-i);
    end
    % Specified values of dydx
    if nder > 0
        for i = 1:n
            A(nfix +(1:nder),i) = (n-i+1)*ones(nder,1).*xder.^(n-i);
        end
    end

    if nfix > 0
        lastcol = n+1;
        nmin = nspec - 1;
    else
        lastcol = n;   % If only derivatives, p(n+1) is arbitrary
        nmin = nspec;
    end
    if n < nmin
        error('Polynomial degree too low. Cannot match all constraints');
    end    
    %% Find the unique polynomial of degree nmin that fits the constraints. 
    firstcol = n-nmin+1;   % A(:,firstcol_lastcol) detrmines p0   
    pc0 = A(:,firstcol:lastcol)\specval;  % Satifies A*pc = specval
    % Now extend to degree n and pad with zeros:
    pc = zeros(n+1,1);
    pc(firstcol:lastcol) = pc0;    % Satisfies A*pcfull = yfix
    
    % Column i in matrix X is the (n-i+1)'th power of x values 
    X = zeros(nfit,n+1);
    for i = 1:n+1
        X(:,i) = ones(nfit,1).*x.^(n+1-i);
    end
        
    % Subtract constraints polynomial values from y. 
    yfit = y-polyval(pc,x);
    
    %% We now find the p0 that mimimises (X*p0-yfit)'*(X*p0-yfit)
    %  given that A*p0 = 0
    B = null(A);    % For any (n+1-nspc by 1) vector z, A*B*z = 0     
    z = X*B\yfit;   % Least squares solution of X*B*z = yfit
    p0 = B*z;       % Satisfies A*p0 = 0;
    p = p0'+pc';    % Satisfies A*p = b; 
end