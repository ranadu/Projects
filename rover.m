% Connect EV3
myev3 = legoev3('USB');

% Define motor ports
motor1 = motor(myev3, 'A');
motor2 = motor(myev3, 'B');

% Set gyro sensor port
gyro = gyroSensor(myev3, 1);
moves_forward(motor1, motor2, gyro);
turn_left(motor1, motor2, gyro);
turn_right(motor1, motor2, gyro);
moves_forward(motor1, motor2, gyro);
turn_right(motor1, motor2, gyro);
small_forward_movement(motor1, motor2, gyro);
turn_left(motor1, motor2, gyro);
moves_forward(motor1, motor2, gyro);

function moves_forward(motor1, motor2, gyro)
% Set initial motor speeds
speed1 = 50; % Motor 1 speed
speed2 = 50; % Motor 2 speed
% Set gyro threshold for motor speed adjustment
gyroThreshold = 3; % degrees
% Reset gyro angle
resetRotationAngle(gyro);
% Run motors for 2.5 seconds
duration = 2.5; % seconds
startTime = tic;
while toc(startTime) < duration
    % Read gyro sensor value
    gyroAngle = readRotationAngle(gyro);
    % Adjust motor speeds based on gyro angle
    if gyroAngle < -gyroThreshold
        speed1 = 60; % Increase Motor 1 speed
        speed2 = 40; % Decrease Motor 2 speed
    elseif gyroAngle > gyroThreshold
        speed1 = 40; % Decrease Motor 1 speed
        speed2 = 60; % Increase Motor 2 speed
    else
        speed1 = 50; % Keep Motor 1 speed
        speed2 = 50; % Keep Motor 2 speed
    end
    % Set motor speeds
    motor1.Speed = speed1;
    motor2.Speed = speed2;
    start(motor1);
    start(motor2);
end

% Stop motors
stop(motor1);
stop(motor2);
% Reset gyro angle
resetRotationAngle(gyro);
end

function small_forward_movement(motor1, motor2, gyro)
% Set initial motor speeds
speed1 = 50; % Motor 1 speed
speed2 = 50; % Motor 2 speed
% Set gyro threshold for motor speed adjustment
gyroThreshold = 3; % degrees
% Reset gyro angle
resetRotationAngle(gyro);
% Run motors for 0.5 seconds
duration = 0.5; % seconds
startTime = tic;
while toc(startTime) < duration
    % Read gyro sensor value
    gyroAngle = readRotationAngle(gyro);
    % Adjust motor speeds based on gyro angle
    if gyroAngle < -gyroThreshold
        speed1 = 60; % Increase Motor 1 speed
        speed2 = 40; % Decrease Motor 2 speed
    elseif gyroAngle > gyroThreshold
        speed1 = 40; % Decrease Motor 1 speed
        speed2 = 60; % Increase Motor 2 speed
    else
        speed1 = 50; % Keep Motor 1 speed
        speed2 = 50; % Keep Motor 2 speed
    end
    % Set motor speeds
    motor1.Speed = speed1;
    motor2.Speed = speed2;
    start(motor1);
    start(motor2);
end
% Stop motors
stop(motor1);
stop(motor2);
% Reset gyro angle
resetRotationAngle(gyro);
end

function turn_right(motor1, motor2, gyro)
% Set motor speeds
speed1 = 50; % Motor 1 speed
speed2 = 0; % Motor 2 speed
% Reset gyro angle
resetRotationAngle(gyro);
% Run motors until gyro reads 90 degrees
while readRotationAngle(gyro) < 90
    motor1.Speed = speed1;
    motor2.Speed = speed2;
    start(motor1);
    start(motor2);
end
% Stop motors
stop(motor1);
stop(motor2);
end

function turn_left(motor1, motor2, gyro)
% Set motor speeds
speed1 = 0; % Motor 1 speed (percentage)
speed2 = 50; % Motor 2 speed (percentage)
% Reset gyro angle
resetRotationAngle(gyro);
% Run motors until gyro reads -30 degrees
while readRotationAngle(gyro) > -90
    motor1.Speed = speed1;
    motor2.Speed = speed2;
    start(motor1);
    start(motor2);
end
% Stop motors
stop(motor1);
stop(motor2);
end
