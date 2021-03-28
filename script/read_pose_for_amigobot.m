function [x, y, t, dt] = read_pose_for_amigobot(rosbag_dir, bot_name, tf)

% https://ww2.mathworks.cn/help/ros/ref/rosbag.html;jsessionid=50ca35ed44dff1683162f6f9a1ea
% https://github.com/ethz-adrl/towr/blob/master/towr/matlab/plot_rosbag.m

    bag = rosbag(rosbag_dir);
    bag_base_pose = select(bag, 'Topic', char('/' + string(bot_name) + '/odom'));
    ts_base_pos = timeseries(bag_base_pose, 'Pose.Pose.Position.X', 'Pose.Pose.Position.Y');

    t = ts_base_pos.Time; %ts_base_pos.Time - ts_base_pos.Time(1);
    dt = ts_base_pos.Time - ts_base_pos.Time(1);
    x_t(:,1)   = ts_base_pos.Data(:,1);
    y_t(:,1)   = ts_base_pos.Data(:,2);
    
    x(:,1) = x_t .* cos(tf(3)) + y_t .* -sin(tf(3)) + tf(1);
    y(:,1) = x_t .* sin(tf(3)) + y_t .*  cos(tf(3)) + tf(2);

    
end