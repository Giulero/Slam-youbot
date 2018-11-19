function [ M ] = rotate(theta)
%ROTATE Summary of this function goes here
%   Detailed explanation goes here
    R =   [cos(theta) -sin(theta) 0;
           sin(theta)  cos(theta) 0;
              0              0    1];
    M = R';
end

