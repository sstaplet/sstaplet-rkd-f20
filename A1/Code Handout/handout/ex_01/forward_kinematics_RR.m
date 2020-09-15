function [ frames ] = forward_kinematics_RR(theta)
% forward_kinematics_RR
%
%   Returns the forward kinematics for an RR robot given
%   the joint angle positions [rad].
%
%   The function returns N 3x3 frames, where N is the
%   combined number of links and joints.
%
%   Each homogeneous transform describes the relationship from
%   the base frame to the corresponding link or joint.
%
%   Hints
%   - 'theta' is a vector. Individual angles can be selected
%      using indices, e.g., theta1 = theta(1)

% length of the links
l1 = 0.375;
l2 = 0.31;

% --------------- BEGIN STUDENT SECTION ----------------------------------
% Define the transforms for all the frames below (replace each 'eye(3)'
% with the corresponding matrix).  You can define helper variable, and
% feel free to express your answers as products of matrices.

theta_1 = theta(1);
theta_2 = theta(2);

H_1_0 = [cos(theta_1) -sin(theta_1) 0; sin(theta_1) cos(theta_1) 0; 0 0 1];
H_2_0 = H_1_0*[1 0 l1; 0 1 0; 0 0 1];
H_3_0 = H_2_0*[cos(theta_2) -sin(theta_2) 0; sin(theta_2) cos(theta_2) 0; 0 0 1];
H_4_0 = H_3_0*[1 0 l2; 0 1 0; 0 0 1];


% --------------- END STUDENT SECTION ------------------------------------

% Pack into a more readable format. DO NOT CHANGE!
frames = cat(3, H_1_0, H_2_0, H_3_0, H_4_0);
end
