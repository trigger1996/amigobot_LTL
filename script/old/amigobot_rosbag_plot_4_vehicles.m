clear
clc
close all

bag_dir = 'C:/Users/ghost/Desktop/ijrr2013-ca1/ijrr2013-ca1-case4.bag';

bot_name = 'amigobot_1';
bot_tf = [0, 3.6, -pi/2];
[x1, y1, t1, dt_1] = read_pose_for_amigobot(bag_dir, bot_name, bot_tf);

bot_name = 'amigobot_2';
bot_tf = [2.4, 0, pi/2];
[x2, y2, t2, dt_2] = read_pose_for_amigobot(bag_dir, bot_name, bot_tf);

bot_name = 'amigobot_3';
bot_tf = [0, 1.2, pi/2];
[x3, y3, t3, dt_3] = read_pose_for_amigobot(bag_dir, bot_name, bot_tf);

bot_name = 'amigobot_4';
bot_tf = [1.8, 3.0, pi/2];
[x4, y4, t4, dt_4] = read_pose_for_amigobot(bag_dir, bot_name, bot_tf);


[dt_1_2, dist_1_2] = get_realtime_reldist(x1, y1, t1, x2, y2, t2);
[dt_1_3, dist_1_3] = get_realtime_reldist(x1, y1, t1, x3, y3, t3);
[dt_1_4, dist_1_4] = get_realtime_reldist(x1, y1, t1, x4, y4, t4);
[dt_2_3, dist_2_3] = get_realtime_reldist(x2, y2, t2, x3, y3, t3);
[dt_2_4, dist_2_4] = get_realtime_reldist(x2, y2, t2, x4, y4, t4);
[dt_3_4, dist_3_4] = get_realtime_reldist(x3, y3, t3, x4, y4, t4);

figure
plot(dt_1_2, dist_1_2)
hold on
plot(dt_1_3, dist_1_3)
hold on
plot(dt_1_4, dist_1_4)
hold on
plot(dt_2_3, dist_2_3)
hold on
plot(dt_2_4, dist_2_4)
%hold on
%plot(dt_3_4, dist_3_4)


