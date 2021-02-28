%% Exercise 1: Introduction to the drawing robot
%% Connect to the hardware
% Once you've set up the Arduino, you can connect to it from MATLAB. You can 
% also connect to the motor carrier, which will be used to communicate with the 
% other peripherals on the robot.

a = arduino;
carrier = motorCarrier(a);
%% Control the servo
% The servo motor is used to raise and lower the whiteboard markers. It should 
% be connected to the SERVO3 port on the carrier. Create a variable in MATLAB 
% to control it.

s = servo(carrier,3);
%% 
% Keep calling writePosition, changing the value by a small amount each time. 
% Explore different values until you figure out which values to use for lowering 
% the left marker, lowering the right marker, and raising both markers.

pos = 0.54; %Change this value and continue running this section
writePosition(s,pos)
%% 
% Once you've identified the servo values for lowering the left and right markers, 
% store them in variables, and save them to a MAT-file. You'll load these values 
% in future exercises that require raising and lowering the markers.

LeftMarker = 0.05;  %Change this to the LeftMarker value on your robot
RightMarker = 0.44; %Change this to the RightMarker value on your robot
NoMarker = mean([LeftMarker RightMarker]);

save ServoPositions.mat LeftMarker RightMarker NoMarker
%% Control the DC motors
% Connect to the DC motors and the motor encoders. These work together to move 
% the robot around the whiteboard. For now, you don't need to hang the robot up. 
% You can leave it lying flat on a table.

mL = dcmotor(carrier,'M2');
mR = dcmotor(carrier,'M1');
eL = rotaryEncoder(carrier,2);
eR = rotaryEncoder(carrier,1);
%% 
% Also, reset the count of the encoders to 0. Now any position will be measured 
% relative to the current motor positions.

resetCount(eL)
resetCount(eR)
%% 
% Next, try controlling the DC motors. The battery included in the kit has a 
% rated voltage of 11.1 V, although the true value may be higher or lower depending 
% on the charge level. You can use a voltmeter to check the exact voltage or use 
% 11.1 as an approximate value.

Vmax = 11.1; %Battery voltage (Volts)
%% 
% Choose a target voltage, |Vset|, that you want to supply to the motors.

Vset = 3; %Target voltage (Volts)
%% 
% The variables mL and mR have a property called |Speed| that controls how fast 
% they run and in which direction. Speed varies between -1 and 1. It creates a 
% PWM signal with an equivalent voltage differential of -V to V across the motor 
% terminals, where V is the supply voltage. Set the |Speed| property based on 
% your target supply voltage.

mL.Speed = Vset/Vmax;
mR.Speed = Vset/Vmax;
%% 
% The motors won't run until you start them. Execute the following code to drive 
% the motors at the target voltage for about three seconds.

start(mL)
start(mR)
pause(3) % Wait 3 seconds
stop(mL)
stop(mR)
%% Read the encoders
% Now that the motors have moved from their starting points, you can check the 
% encoder values to see how far they've been displaced since you last reset the 
% counts. In the next lesson, you'll learn how to translate this value into physical 
% distance units.

count1 = readCount(eL)
count2 = readCount(eR)
%% Closed-loop control
% In this section, you will see how to achieve closed-loop position control, 
% without the need to create the encoder variables eL and eR. First, you need 
% to clear any existing connections to the DC motors.

clear mL mR eL eR
%% 
% Configure a PID controller for the closed-loop control of the DC motors. From 
% the motor specification sheet, we see that the no. of pulses per revolution 
% of the encoder is 3. Based on empirical results, we found the gain values Kp 
% = 0.18, Ki = 0.0, and Ki = 0.01 to be suitable for this exercise.

pidML = pidMotor(carrier,2,'position',3,[0.18 0.0 0.01]); % Modify the PID gains [Kp Ki Kd] as per your requirements
pidMR = pidMotor(carrier,1,'position',3,[0.18 0.0 0.01]); % Modify the PID gains [Kp Ki Kd] as per your requirements
%% 
% The motor specification sheet tells us that the gear ratio of the motors is 
% 100. Set a target angular displacement value for the PID controller in radians.

gearRatio = 100;
theta = 5*2*pi*gearRatio;   % 5 revolutions of the motor in radians
%% 
% The methods of the pidMotor object automatically start and stop the motor 
% in position control mode after achieving the target angular displacement value. 
% You can set the mode of calculation of the angle of rotation to either absolute 
% (abs) or relative (rel). Execute the following code to command the motors to 
% complete 5 revolutions about their axis.

writeAngularPosition(pidML,theta,'rel');
writeAngularPosition(pidMR,theta,'rel');
%% Read the controlled variables
% You can verify the accuracy of the controller by reading the end angular displacement 
% value.

readAngularPosition(pidML)
readAngularPosition(pidMR)
%% 
% Always clear hardware variables whenever you're done using them.

clear a carrier s pidML pidMR
%% 
% _Copyright 2018 - 2020 The MathWorks, Inc._