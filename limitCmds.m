function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,WHEEL2CENTER) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       wheel2Center distance from the wheels to the center of the robot(in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   SAJAN, NAYANTHARA


wheel1V = fwdVel + angVel*wheel2Center;
wheel2V = fwdVel - angVel*wheel2Center;

if abs(wheel1V)>=abs(wheel2V)
    scaleV = abs(wheel1V);
else
    scaleV = abs(wheel2V);
end

if scaleV>maxV
    cmdV = (fwdVel/scaleV)*maxV;
    cmdW = (angVel/scaleV)*maxV;
else
    cmdV = fwdVel;
    cmdW = angVel;
end

end
