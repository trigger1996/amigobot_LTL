function [position] = read_trajectories_for_amigobot(rosbag_dir, bot_name)

% https://ww2.mathworks.cn/help/ros/ref/rosbag.html;jsessionid=50ca35ed44dff1683162f6f9a1ea
% https://github.com/ethz-adrl/towr/blob/master/towr/matlab/plot_rosbag.m

    bag  = rosbag(rosbag_dir);
    bSel = select(bag, 'Topic', char('/' + string(bot_name) + '/path'));
    data = readMessages(bSel, bSel.NumMessages);  

    pathlength = length(data{1}.Poses);
    for i = 1 : pathlength
        position(i,:)=[data{1}.Poses(i).Pose.Position.X data{1}.Poses(i).Pose.Position.Y];
    end
   
end