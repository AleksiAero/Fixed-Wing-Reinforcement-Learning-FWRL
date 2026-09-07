function fwrl_integration_check()
% Run with WSL simulator and python_controller:=false, domain 42.
n = ros2node('/fwrl_integration_check',42);
s = ros2subscriber(n,'/fwrl/odometry','nav_msgs/Odometry',Depth=1);
before = receive(s,10);
fwrl_controller(3,42);
after = receive(s,10);
p0 = before.pose.pose.position;
p1 = after.pose.pose.position;
distance = norm([p1.x-p0.x,p1.y-p0.y,p1.z-p0.z]);
assert(distance > 10, 'MATLAB commands did not move WSL simulator');
fprintf('Windows MATLAB / WSL ROS 2 closed loop passed: %.2f m movement\n',distance);
end
