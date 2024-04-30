%% Initialize
myev3 = legoev3('USB');
link1_motor = motor(myev3, 'A');
link2_motor = motor(myev3, 'B');
link1_sensor = touchSensor(myev3,1);
link2_sensor = touchSensor(myev3,2);
pen_motor = motor(myev3, 'C');


%% put pen lift here 
pen_up(pen_motor)
disp('A')
pause(2)

%% put homing mechanism here
home_link1(link1_sensor, link1_motor);
disp('B')
pause(4)
home_link2(link2_sensor, link2_motor);
disp('C')

%% Scaling factors and origin points for workspace and link lengths 
kx = 13; %change based on size of workspace x axis vs size of 1
ky = 13; %change based on size of workspace y axis vs size of 1
o_x = -5; %change based on location of link base to corner of workspace
o_y = 10; %change based on location of link base to corner of workspace
L1 = 13; %change based on measured value 
L2 = 10; %change based on measured value 
gear_ratio = 3;
Speed1 = 50;
Speed2 = 50;

%% Activate pen.m
pen
disp('draw something')

pause(10);


%% extract coordinates to determine required thetas
x_extract = coordinates.x(~isnan(coordinates.x)); %removes NaN points
y_extract = coordinates.y(~isnan(coordinates.y)); %removes NaN points
x = kx*x_extract + o_x;
y = ky*y_extract + o_y;
theta = inv_kinematics(x,y,L1,L2);

resetRotation(link1_motor)
resetRotation(link2_motor)

%% call plotting function
plotRobotArm(theta, L1, L2)


% need to integrate pen down after the robot has left home and reach
%% coordinate 1 
previoustheta1 = 0;
previoustheta2 = 0;
current_theta1 = rad2deg(theta(1,1));
current_theta2 = rad2deg(theta(2,1));
move(link1_motor, link2_motor, current_theta1, current_theta2, Speed1, Speed2, gear_ratio, previoustheta1, previoustheta2)
previoustheta1 = rad2deg(theta(1,1));
previoustheta2 = rad2deg(theta(2,1));

% put pen down here
pen_down(pen_motor)

nRows = size(theta, 2);
for i = 2:nRows
    current_theta1 = rad2deg(theta(1,i));
    current_theta2 = rad2deg(theta(2,i));

    move(link1_motor, link2_motor, current_theta1, current_theta2, Speed1, Speed2, gear_ratio, previoustheta1, previoustheta2)
    previoustheta1 = rad2deg(theta(1,i));
    previoustheta2 = rad2deg(theta(2,i));
end

% Inverse kinematics
function theta = inv_kinematics(x,y,L1,L2)
    theta2 = acos((x.^2 + y.^2 - L1^2 - L2^2)/(2*L1*L2));
    theta1 = atan2(y,x) - atan2((L2*sin(theta2)),(L1 + L2*cos(theta2)));
    theta = [theta1; theta2];
end

% Simulated plot of movement 
function plotRobotArm(theta, L1, L2)
    % Number of steps in the drawing
    nSteps = size(theta, 2);
    
    figure; % Create a new figure window
    axis equal; % Keep aspect ratio of the plot square
    grid on; % Turn on the grid for better visualization
    hold on; % Keep the plot from erasing previous lines
    xlim([-L2, L1+L2]); % Set x-axis limits
    ylim([-L2, L1+L2]); % Set y-axis limits
    xlabel('X');
    ylabel('Y');
    title('2-Link Planar Robot Movement');
    
    % Loop through each theta set and plot the robot arm configuration
    for i = 1:nSteps
        % Extract joint angles
        theta1 = theta(1,i);
        theta2 = theta(2,i);
        
        % Calculate the joint positions
        joint1_x = L1 * cos(theta1);
        joint1_y = L1 * sin(theta1);
        end_effector_x = joint1_x + L2 * cos(theta1 + theta2);
        end_effector_y = joint1_y + L2 * sin(theta1 + theta2);
        
        % Clear the current plot
        cla;
        
        % Plot the robot arm
        plot([0, joint1_x], [0, joint1_y], 'r', 'LineWidth', 2); % Link 1
        plot([joint1_x, end_effector_x], [joint1_y, end_effector_y], 'b', 'LineWidth', 2); % Link 2
        plot(end_effector_x, end_effector_y, 'ko', 'MarkerFaceColor', 'k'); % End effector (pen)
        
        pause(0.1); % Pause to visually follow the arm movement
    end
    
    hold off; % Release plot hold
end

% pen up function
function pen_up(pen_motor)
resetRotation(pen_motor);
pen_motor.Speed = 50;%Change to right direction
currentRotation = readRotation(pen_motor);
start(pen_motor);
while abs(currentRotation)<160%2 rev (might need to change to rads)
   currentRotation = readRotation(pen_motor);
end
stop(pen_motor);
end

% pen down function
function pen_down(pen_motor)
resetRotation(pen_motor);
pen_motor.Speed = -50;%Change to right direction
currentRotation = readRotation(pen_motor);
start(pen_motor);
while abs(currentRotation)<160%2 rev (might need to change to rads)
   currentRotation = readRotation(pen_motor);
end
stop(pen_motor);
end

% homing link 1 function 
function home_link1(link1_sensor, link1_motor)
Homedis1 = readTouch(link1_sensor);
   while Homedis1 ~=1
link1_motor.Speed = -40;%change---------------
start(link1_motor);
Homedis1 = readTouch(link1_sensor);
       if Homedis1 == 1
   stop(link1_motor)
   break
       end
   end
end

% homing link 2 function 
function home_link2(link2_sensor, link2_motor)
Homedis2 = readTouch(link2_sensor);
   while Homedis2 ~=1
link2_motor.Speed = -20;%change---------------
start(link2_motor);
Homedis2 = readTouch(link2_sensor);
       if Homedis2 == 1
   stop(link2_motor)
   break
       end
   end
end


% move motors for each theta increment based on extracted coordinates
function move(link1_motor, link2_motor, current_theta1, current_theta2, Speed1, Speed2, gear_ratio, previoustheta1, previoustheta2)
    % Calculate the difference in theta to determine the direction
    theta1_diff = current_theta1 - previoustheta1;
    theta2_diff = current_theta2 - previoustheta2;

    % Convert theta differences to target rotations considering the gear ratio
    target_rotation1 = abs(theta1_diff) / gear_ratio;
    target_rotation2 = abs(theta2_diff) / gear_ratio;

    % Determine the direction of the rotation based on the sign of the theta difference
     if theta1_diff > 0 
        link1_motor.Speed = Speed1;
    elseif theta1_diff < 0
        link1_motor.Speed = -Speed1;
    end 
    if theta2_diff > 0 
        link2_motor.Speed = Speed2;
    elseif theta2_diff < 0
        link2_motor.Speed = -Speed2;
    end 

    % Reset the motor rotation counters to zero
    resetRotation(link1_motor);
    resetRotation(link2_motor);

    % Start the motors
    start(link1_motor);
    start(link2_motor);

    % Wait for each motor to reach its target rotation
    while true
        if abs(readRotation(link1_motor)) >= abs(target_rotation1) && abs(readRotation(link2_motor)) >= abs(target_rotation2)
            stop(link1_motor, 1); % Use active braking to stop the motor
            stop(link2_motor, 1); % Use active braking to stop the motor
            break; % Exit the loop once both motors have reached their target
        end
    end
end
