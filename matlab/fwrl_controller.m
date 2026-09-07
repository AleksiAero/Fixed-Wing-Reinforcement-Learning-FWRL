function fwrl_controller(durationSeconds, domainId)
% Waypoint-navigation reference controller for the idealized FWRL simulator.
% Run ROS launch with python_controller:=false. World: ENU; body: FLU.
% Commands are bank/gamma/speed setpoints, not actuator deflections.
arguments
    durationSeconds (1,1) double {mustBePositive} = 600
    domainId (1,1) double {mustBeInteger,mustBeNonnegative} = 42
end
node = ros2node('/fwrl_matlab_controller', domainId);
odom = ros2subscriber(node, '/fwrl/odometry', 'nav_msgs/Odometry', Reliability='reliable', Depth=1);
target = ros2subscriber(node, '/fwrl/waypoint', 'geometry_msgs/PointStamped', Reliability='reliable', Depth=1);
pub = ros2publisher(node, '/fwrl/command', 'geometry_msgs/TwistStamped', Reliability='reliable', Depth=1);
msg = ros2message(pub);
msg.header.frame_id = 'fwrl_setpoint';
start = tic;
while toc(start) < durationSeconds
    try
        t = receive(target, 1);
        s = receive(odom, 1);
    catch
        % No commands are emitted when observations stop arriving.
        pause(.05);
        continue
    end
    if ~strcmp(s.header.frame_id,'map') || ~strcmp(t.header.frame_id,'map')
        continue
    end
    p = s.pose.pose.position;
    q = s.pose.pose.orientation;
    yaw = atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
    delta = [t.point.x-p.x; t.point.y-p.y; t.point.z-p.z];
    error = atan2(sin(atan2(delta(2),delta(1))-yaw), cos(atan2(delta(2),delta(1))-yaw));
    msg.header.stamp = s.header.stamp;
    msg.twist.angular.x = min(.7, max(-.7, 1.2*error));
    msg.twist.angular.y = min(.25, max(-.25, atan2(delta(3), max(norm(delta(1:2)),30))));
    msg.twist.linear.x = 22;
    send(pub, msg);
end
end
