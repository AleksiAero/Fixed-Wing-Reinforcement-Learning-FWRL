function fwrl_check()
% Local installation checks; network discovery is verified separately.
assert(~isempty(ver('ros')), 'ROS Toolbox is missing');
assert(license('test','ROS_Toolbox'), 'ROS Toolbox license unavailable');
n = ros2node('/fwrl_matlab_check', 42);
p = ros2publisher(n, '/fwrl/check', 'std_msgs/String');
m = ros2message(p);
m.data = 'FWRL MATLAB ROS 2 ready';
send(p,m);
disp(version);
disp('MATLAB ROS 2 node and publisher created successfully');
end
