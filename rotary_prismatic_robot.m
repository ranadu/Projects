myev3 = legoev3('USB');

%% Prismatic Joint
% Define motor and sensor ports
motorPort = 'A';
inputport = 1; % color sensor is connected to port 1

% Attach motor
motorb = motor(myev3, motorPort);
angle1 = readRotation(motorb);
mytouchsensor = touchSensor(myev3,inputport);

% Set up gear ratio
gearRadian = 20; % millimeters

while true
    % Function to move to a specified angle
    resetRotation(motorb);
    
    % Homing Routine
    % Move the motor until the touch sensor is pressed
    motor.Speed = 5; % Adjust speed as needed
    start(motorb);
    while true
         if readTouch(mytouchsensor)
            % If touch sensor is pressed, stop the motor and break the loop
            stop(motorb);
            break;
        end
    end
    
    % Reset rotation
    resetRotation(motorb);
    
    % Input dialog box for distance input
    % Create a prompt for the input dialog
    prompt = {'Enter the distance:'};
    
    % Set the title for the dialog
    dlgtitle = 'Distance Input from 0 - 50mm';
    
    % Set the dimensions of the input box
    dims = [1 35];
    
    % Default input value (if any)
    definput = {'0'};
    
    % Display the input dialog and get the user input
    distInput = inputdlg(prompt, dlgtitle, dims, definput);
    
    % Check if the user clicked 'Cancel'
    if isempty(distInput)
        disp('User canceled the operation.');
        break; % Exit the loop if user cancels
    else
        % Convert the input string to a numeric value (assuming it's a number)
        dist = str2double(distInput{1});

        theta_1 = dist/gearRadian;
        theta_2 = rad2deg(theta_1)
        % Call the function with the provided dist
        moveToDistance(motorb, theta_2);
    end
end

    function moveToDistance(motor, theta_2)
    
    % Calculate target position based on gear ratio
    targetPositionD = theta_2;
    % Set motor position
    motor.Speed = 5; % Adjust speed as needed

    % Start motor
    start(motor);
    % Wait for the motor to reach the target position
    while true
        angleMotor = readRotation(motor);
        if abs(angleMotor) >= abs(targetPositionD)
            break
        end
    end
    % Stop motor
    stop(motor, 1);
    pause(10);
    end



%% Rotary Joint
% Define motor and sensor ports
motorPort = 'D';
inputport = 4; % color sensor is connected to port 1

% Attach motor
myev3 = legoev3('USB');
motorb = motor(myev3, motorPort);
angle1 = readRotation(motorb);
mytouchsensor = touchSensor(myev3,inputport);


% Set up gear ratio
gearRatio = 2.5; % Adjust based on your robot's gear configuration

while true
    % Function to move to a specified angle
    resetRotation(motorb);
    
    % Homing Routine
    % Move the motor until the touch sensor is pressed
    motor.Speed = 10; % Adjust speed as needed
    start(motorb);
    while true
         if readTouch(mytouchsensor)
            % If touch sensor is pressed, stop the motor and break the loop
            stop(motorb);
            break;
        end
    end
    
    % Reset rotation
    resetRotation(motorb);
    
    % Input dialog box for angle input
    % Create a prompt for the input dialog
    prompt = {'Enter the angle:'};
    
    % Set the title for the dialog
    dlgtitle = 'Angle Input';
    
    % Set the dimensions of the input box
    dims = [1 35];
    
    % Default input value (if any)
    definput = {'0'};
    
    % Display the input dialog and get the user input
    angleInput = inputdlg(prompt, dlgtitle, dims, definput);
    
    % Check if the user clicked 'Cancel'
    if isempty(angleInput)
        disp('User canceled the operation.');
        break; % Exit the loop if user cancels
    else
        % Convert the input string to a numeric value (assuming it's a number)
        angle = str2double(angleInput{1});
        
        % Call the function with the provided angle
        moveToAngle(motorb, angle, gearRatio);
    end
end

% Function to move to a specified angle

function moveToAngle(motor, targetAngle, gearRatio)
    % Calculate target position based on gear ratio
    targetPosition = targetAngle * gearRatio;
    % Set motor position
    motor.Speed = 30; % Adjust speed as needed
    % Start motor
    start(motor);
    % Wait for the motor to reach the target position
    while true
        angleMotor = readRotation(motor);
        if abs(angleMotor) >= abs(targetPosition)
            break
        end
    end
    % Stop motor
    stop(motor, 1);
    pause(10);
    % start (motor);
    % motor.Speed = -30;
    %  while true
    %     angleMotor = readRotation(motor);
    %     if angleMotor == -(targetPosition)
    %         break
    %     end
    % end
    % 
    % stop (motor,1);
end

