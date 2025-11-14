function expmat = ECE569_MatrixLog6(T)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a transformation matrix T in SE(3).
% Returns the corresponding se(3) representation of exponential 
% coordinates.
% Example Input:
% 
% clear; clc;
% T = [[1, 0, 0, 0]; [0, 0, -1, 0]; [0, 1, 0, 3]; [0, 0, 0, 1]];
% expmat = MatrixLog6(T)
% 
% Output:
% expc6 =
%         0         0         0         0
%         0         0   -1.5708    2.3562
%         0    1.5708         0    2.3562
%         0         0         0         0

[R, p] = ECE569_TransToRp(T);
omgmat = ECE569_MatrixLog3(R);
if isequal(omgmat, zeros(3))
    expmat = [zeros(3),p;zeros(1,4)];
else
    theta = acos((trace(R) - 1)/2);
    omgmat_unit= (omgmat / theta);
    % From Modern Robotics (3.92)
    Vinv     = eye(3)/theta ...
            - 0.5 * omgmat_unit ...
            + (1/theta - 0.5*cot(theta/2)) * (omgmat_unit*omgmat_unit);
    v = Vinv * p;
    expmat = [omgmat,v*theta;zeros(1,4)];
end
end