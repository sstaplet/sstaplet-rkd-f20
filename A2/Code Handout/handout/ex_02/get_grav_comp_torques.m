function [ torque ] = get_grav_comp_torques(theta, gravity)
% get_grav_comp_torques
%
%   Calculates the joint torques required to cancel out effects due to
%   gravity.

% Get information about the robot:
robot = robot_info();
% Extract mass of the links, joint, and end effector [kg]
m_link_1 = robot.link_masses(1);
m_link_2 = robot.link_masses(2);
m_joint_1 = robot.joint_masses(1);
m_joint_2 = robot.joint_masses(2);
m_end_effector = robot.end_effector_mass;

% --------------- BEGIN STUDENT SECTION ----------------------------------
% Use the Jacobian to calculate the joint torques to compensate for the
% weights of the joints, links, and end effector (assuming the acceleration
% due to gravity is given be 'gravity', and it is a 2x1 (column) vector).

fl1 = [-m_link_1*gravity];
fl2 = [-m_link_2*gravity];
fj1 = [-m_joint_1*gravity];
fj2 = [-m_joint_2*gravity];
meE = [-m_end_effector*gravity];

jBoiCom = jacobian_coms_RR(theta);
m_1_j = jBoiCom(:,:,1);
m_2_j = jBoiCom(:,:,2);
  
jBoiLink = jacobian_link_ends_RR(theta);
l_1_j = jBoiLink(:,:,1);
l_2_j = jBoiLink(:,:,2);

tL1 = transpose(m_1_j)*fl1;
tL2 = transpose(m_2_j)*fl2;
tJ1 = [0 0; 0 0]*fj1;
tJ2 = transpose(l_1_j)*fj2;
tMeE = transpose(l_2_j)*meE;
torqueTotal = tL1 + tL2 + tJ1 + tJ2 + tMeE;

torque1 = torqueTotal(1);
torque2 = torqueTotal(2);

% --------------- END STUDENT SECTION ------------------------------------
% Pack into a more readable format. DO NOT CHANGE!
torque = cat(1, torque1, torque2);
end
