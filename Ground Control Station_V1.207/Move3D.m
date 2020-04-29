function out = Move3D(ARc, vx, vy, vz, vr)

% Command velocities
v = [-vy*0.2, -vx*0.2, vz*1.0, -vr*0.5];
mode = (abs(v(1)) > 0.0 || abs(v(2)) > 0.0 || abs(v(3)) > 0.0 || abs(v(4)) > 0.0);

% Nomarization (-1.0 to +1.0)
for i = 1:4
    if (abs(v(i)) > 1.0)
        v(i) = v(i)/abs(v(i));
    end
end
ARYaw   = sprintf('AT*PCMD=%d,%d,%d,%d,%d,%d\r',tic,mode,ARDrone_FloatArg2Int(v(1)), ...
    ARDrone_FloatArg2Int(v(2)),ARDrone_FloatArg2Int(v(3)),ARDrone_FloatArg2Int(v(4)));
fprintf(ARc, ARYaw);
