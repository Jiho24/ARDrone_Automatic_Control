% demo 3
%  this demo let the drone fly along a specific trajectory clockwisely.
%    the trajectory looks like as following:
% 
%             --|
%            |  |
%          |    |
%        |      |
% start|        |
%               |
%               |
%  end |        |
%      |        |
%      ----------
%  
%  the path is set in the beginning of the demo code.
% Note: for the default trajectory demo needs a 2meter * 2meter (at least) empty room
% 
% *************************************
% *  Authors:
%    Kun Zhang (dabiezu@gmail.edu)
%    Pieter J. Mosterman (pmosterman@yahoo.com) *
% *************************************
% 

clc
close all
clear all



% WayPoint data structure:
%   each row: [displacement in x, displacement in y, orientation, duration of execution]
% the target orientation has no use currently, since the controller does not take it into consideration
% All Waypoint is defined in the Drone body-coordinate
% if duration of execution is not large enough, the drone can not reach the target point since it will
% time out and stop execution
% if duration is too large, the drone will takes too much time to tune its position after arrival.
%
% The drone forward direction is x-axis, while its perpendicular direction is y-axis.

WayPoint = [0.6, 1.5, 193, 3
            -1.5, 0, 193, 4
             0, -1.5, 193, 4
             0.9, 0, 193, 3];

         
         
%  ----------------------------
addpath('./ARDroneMatlabAPI');
PointNum = size(WayPoint);
PointNum = PointNum(1);


controlChannel = udp('192.168.1.1', 5556, 'LocalPort', 5556);
stateChannel = udp('192.168.1.1', 5554, 'LocalPort', 5554);
fopen(controlChannel);
fopen(stateChannel);
SequenceNumber = tic;

SequenceNumber = TakeOff(SequenceNumber, controlChannel, stateChannel);

if SequenceNumber ~= -1
    
    
pause(5);


%   1      2       3         4          5          6
%  flag,LR_tilt,FB_tilt,VerticalVel,AngularVel  command duration

[SequenceNumber ~] = TrackRelativeDisplacement ( SequenceNumber, controlChannel, stateChannel, WayPoint,0, 'indoor');

pause(5);

SequenceNumber = Land(SequenceNumber, controlChannel,stateChannel);

end


fclose(controlChannel);
fclose(stateChannel);






    






