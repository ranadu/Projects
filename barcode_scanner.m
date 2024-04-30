% Initialize EV3 connection
ev3 = legoev3();

% Set up motor and sensors
motor1 = motor(ev3, 'A'); % Conveyor belt motor
motor2 = motor(ev3, 'B'); % Swing arm motor
sensor_c = colorSensor(ev3, 1); % Color sensor
sensor_u = sonicSensor(ev3, 2); % Ultrasonic sensor

% Set initial conveyor belt speed
start(motor1, 5);

                code1 = {'Red', 'Black', 'White', 'Blue'};
                code2 = {'Red', 'Black', 'Blue', 'White'};
                code3 = {'Red', 'White', 'Black', 'Blue'};
                code4 = ['Red', 'White', 'Blue', 'Black'];
                code5 = {'Red', 'Blue', 'Black', 'White'};
                code6 = {'Red', 'Blue', 'White', 'Black'};
                code7 = {'Black', 'Red', 'White', 'Blue'};
                code8 = {'Black', 'Red', 'Blue', 'White'};
                code9 = {'Black', 'White', 'Red', 'Blue'};
                code10 ={'Black', 'White', 'Blue', 'Red'};
                code11 ={'Black', 'Blue', 'Red', 'White'};
                code12 ={'Black', 'Blue', 'White', 'Red'};
                code13 ={'White', 'Red', 'Black', 'Blue'};
                code14 ={'White', 'Red', 'Blue', 'Black'};
                code15 ={'White', 'Black', 'Red', 'Blue'};
                code16 ={'White', 'Black', 'Blue', 'Red'};
                code17 ={'White', 'Blue', 'Red', 'Black'};
                code18 ={'White', 'Blue', 'Black', 'Red'};
                code19 ={'Blue', 'Red', 'Black', 'White'};
                code20 ={'Blue', 'Red', 'White', 'Black'};
                code21 =['Blue', 'Black', 'Red', 'White'];
                code22 ={'Blue', 'Black', 'White', 'Red'};
                code23 ={'Blue', 'White', 'Red', 'Black'};
                code24 ={'Blue', 'White', 'Black', 'Red'};
                 

% Create an empty array to store the colors
colours = {};

% Main loop
while true
    % % Loop to read four colors
    % while numel(colours) < 4
    %     % Read color from the sensor
    %     color = readColor(sensor_c);
    % 
    %     % Add color to the array if it's not already in there
    %     if ~ismember(color, colours)
    %         colours = [colours color];
    %     end
    % end
    % 
colourSQ = code21 %Assuming that it scanned the colour code 21
    
    switch colourSQ
        case code4
            moveAndRotateArm(motor1, motor2, sensor_u, 0.0380, 45); % Angle for code4
            break;

        case {code2, code18}
            moveAndRotateArm(motor1, motor2, sensor_u, 0.1130, 45); % Angle for code2 or code18
            break;

        case code21
            moveAndRotateArm(motor1, motor2, sensor_u, 0.0380, 90); % Angle for code21
            break;

        case {code7, code23} %Angle for code7 or code23
            % Move conveyor belt in opposite direction for 3 seconds
            start(motor1, -15);
            pause(5);
            stop(motor1);
            break;
        
        % otherwise
        %     % Stop conveyor belt for 3 seconds and display error message
        %     stop(motor1);
        %     pause(3);
        %     writeLCD(ev3, 'Cannot scan, please rescan');
    end
    
    disp('Test')
    
    % Clear colors array for next scan
    colours = [];
end


% Function to move conveyor belt and swing arm

function moveAndRotateArm(motor1, motor2, sensor_u, mysensor, angle)
    % Move conveyor belt until ultrasonic sensor reads the specified distance
    while readDistance(sensor_u) > mysensor % Adjust this value accordingly
        start(motor1, 5);
    end
    stop(motor1);

    % Swing arm to specified angle and return to original position
    resetRotation(motor2)
    motor2.Speed = 15;
    start(motor2)
    while(readRotation(motor2)) < angle
    end
    stop(motor2,1)

    motor2.Speed = -20;

    resetRotation(motor2)
    pause(1)

    start(motor2)

    while(readRotation(motor2)) > -angle
        readRotation(motor2)
    end
    stop(motor2,1)
end

