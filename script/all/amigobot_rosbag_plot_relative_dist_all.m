clear
clc
close all

%% Case 1 & 2

bag_dir = 'E:/IJRR2013-CA1/ijrr2013-ca1-case1&2.bag';

bot_name = 'amigobot_1';
bot_tf = [0, 3.6, -pi/2];
[x1_c1, y1_c1, t1_c1, dt_1_c1] = read_pose_for_amigobot(bag_dir, bot_name, bot_tf);

bot_name = 'amigobot_2';
bot_tf = [2.4, 0, pi/2];
[x2_c1, y2_c1, t2_c1, dt_2_c1] = read_pose_for_amigobot(bag_dir, bot_name, bot_tf);

bot_name = 'amigobot_3';
bot_tf = [1.8, 0.6, -pi];
[x3_c1, y3_c1, t3_c1, dt_3_c1] = read_pose_for_amigobot(bag_dir, bot_name, bot_tf);

[dt_1_2_c1, dist_1_2_c1] = get_realtime_reldist(x1_c1, y1_c1, t1_c1, x2_c1, y2_c1, t2_c1);
[dt_1_3_c1, dist_1_3_c1] = get_realtime_reldist(x1_c1, y1_c1, t1_c1, x3_c1, y3_c1, t3_c1);
[dt_2_3_c1, dist_2_3_c1] = get_realtime_reldist(x2_c1, y2_c1, t2_c1, x3_c1, y3_c1, t3_c1);

[mindist_1_2_c1, index_1_2_c1] = min(dist_1_2_c1);
[mindist_1_3_c1, index_1_3_c1] = min(dist_1_3_c1);
[mindist_2_3_c1, index_2_3_c1] = min(dist_2_3_c1);


%%
% Case 3
bag_dir = 'E:/IJRR2013-CA1/ijrr2013-ca1-case3.bag';

bot_name = 'amigobot_1';
bot_tf = [0, 3.6, -pi/2];
[x1_c3, y1_c3, t1_c3, dt_1_c3] = read_pose_for_amigobot(bag_dir, bot_name, bot_tf);

bot_name = 'amigobot_2';
bot_tf = [2.4, 0, pi/2];
[x2_c3, y2_c3, t2_c3, dt_2_c3] = read_pose_for_amigobot(bag_dir, bot_name, bot_tf);

bot_name = 'amigobot_3';
bot_tf = [1.8, 0.6, -pi];
[x3_c3, y3_c3, t3_c3, dt_3_c3] = read_pose_for_amigobot(bag_dir, bot_name, bot_tf);

[dt_1_2_c3, dist_1_2_c3] = get_realtime_reldist(x1_c3, y1_c3, t1_c3, x2_c3, y2_c3, t2_c3);
[dt_1_3_c3, dist_1_3_c3] = get_realtime_reldist(x1_c3, y1_c3, t1_c3, x3_c3, y3_c3, t3_c3);
[dt_2_3_c3, dist_2_3_c3] = get_realtime_reldist(x2_c3, y2_c3, t2_c3, x3_c3, y3_c3, t3_c3);

[mindist_1_2_c3, index_1_2_c3] = min(dist_1_2_c3);
[mindist_1_3_c3, index_1_3_c3] = min(dist_1_3_c3);
[mindist_2_3_c3, index_2_3_c3] = min(dist_2_3_c3);

%%
bag_dir = 'E:/IJRR2013-CA1/ijrr2013-ca1-case4.bag';

bot_name = 'amigobot_1';
bot_tf = [0, 3.6, -pi/2];
[x1_c4, y1_c4, t1_c4, dt_1_c4] = read_pose_for_amigobot(bag_dir, bot_name, bot_tf);

bot_name = 'amigobot_2';
bot_tf = [2.4, 0, pi/2];
[x2_c4, y2_c4, t2_c4, dt_2_c4] = read_pose_for_amigobot(bag_dir, bot_name, bot_tf);

bot_name = 'amigobot_3';
bot_tf = [0, 1.2, pi/2];
[x3_c4, y3_c4, t3_c4, dt_3_c4] = read_pose_for_amigobot(bag_dir, bot_name, bot_tf);

bot_name = 'amigobot_4';
bot_tf = [1.8, 3.0, pi/2];
[x4_c4, y4_c4, t4_c4, dt_4_c4] = read_pose_for_amigobot(bag_dir, bot_name, bot_tf);


[dt_1_2_c4, dist_1_2_c4] = get_realtime_reldist(x1_c4, y1_c4, t1_c4, x2_c4, y2_c4, t2_c4);
[dt_1_3_c4, dist_1_3_c4] = get_realtime_reldist(x1_c4, y1_c4, t1_c4, x3_c4, y3_c4, t3_c4);
[dt_1_4_c4, dist_1_4_c4] = get_realtime_reldist(x1_c4, y1_c4, t1_c4, x4_c4, y4_c4, t4_c4);
[dt_2_3_c4, dist_2_3_c4] = get_realtime_reldist(x2_c4, y2_c4, t2_c4, x3_c4, y3_c4, t3_c4);
[dt_2_4_c4, dist_2_4_c4] = get_realtime_reldist(x2_c4, y2_c4, t2_c4, x4_c4, y4_c4, t4_c4);
[dt_3_4_c4, dist_3_4_c4] = get_realtime_reldist(x3_c4, y3_c4, t3_c4, x4_c4, y4_c4, t4_c4);

[mindist_1_2_c4, index_1_2_c4] = min(dist_1_2_c4);
[mindist_1_3_c4, index_1_3_c4] = min(dist_1_3_c4);
[mindist_1_4_c4, index_1_4_c4] = min(dist_1_4_c4);
[mindist_2_3_c4, index_2_3_c4] = min(dist_2_3_c4);
[mindist_2_4_c4, index_2_4_c4] = min(dist_2_4_c4);
[mindist_3_4_c4, index_3_4_c4] = min(dist_3_4_c4);

%%
figure('color',[1 1 1])
set(gcf,'outerposition',get(0,'screensize'));

subplot(3,1,1)
plot(dt_1_2_c1, dist_1_2_c1, 'linewidth', 1.8, 'color', [36, 169, 225]/255)
hold on
plot(dt_1_3_c1, dist_1_3_c1, 'linewidth', 1.8, 'color', [29, 191, 151]/255)
hold on
plot(dt_2_3_c1, dist_2_3_c1, 'linewidth', 1.8, 'color', [254, 67, 101]/255)

% Mark the minium value for Case 1 & 2
text(dt_1_2_c1(index_1_2_c1 - 35),mindist_1_2_c1 + 0.1,'o','color', [255, 94, 72]/255, 'FontSize',15)
text(dt_1_2_c1(index_1_2_c1 + 25), mindist_1_2_c1 - 0.2, ...
     ['(',num2str(dt_1_2_c1(index_1_2_c1), '%.2f'),',',num2str(mindist_1_2_c1, '%.2f'),')'], ...
     'color', 'k', 'Interpreter', 'latex', 'FontSize',8);
text(dt_1_3_c1(index_1_3_c1 - 45),mindist_1_3_c1 + 0.1,'o','color', [255, 94, 72]/255, 'FontSize',15)
text(dt_1_3_c1(index_1_3_c1 + 25), mindist_1_3_c1 - 0.2, ...
     ['(',num2str(dt_1_3_c1(index_1_3_c1), '%.2f'),',',num2str(mindist_1_3_c1, '%.2f'),')'], ...
     'color', 'k', 'Interpreter', 'latex', 'FontSize',8);
text(dt_2_3_c1(index_2_3_c1 - 25),mindist_2_3_c1 + 0.1,'o','color', [255, 94, 72]/255, 'FontSize',15)
text(dt_2_3_c1(index_2_3_c1 + 25), mindist_2_3_c1 - 0.2, ...
     ['(',num2str(dt_2_3_c1(index_2_3_c1), '%.2f'),',',num2str(mindist_2_3_c1, '%.2f'),')'], ...
     'color', 'k', 'Interpreter', 'latex', 'FontSize',8);
 
axis([0, 1300, 0 5])
set(gca, 'linewidth', 1.1, 'fontsize', 16)
set(gca,'TickLabelInterpreter','latex') 

%xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize',24)
%ylabel('Relative Distance (m)', 'Interpreter', 'latex', 'FontSize',24)

set(gca,'xgrid','on')
set(gca,'ygrid','on')
set(gca,'Box','off')


subplot(3,1,2)
plot(dt_1_2_c3, dist_1_2_c3, 'linewidth', 1.8, 'color', [36, 169, 225]/255)
hold on
plot(dt_1_3_c3, dist_1_3_c3, 'linewidth', 1.8, 'color', [29, 191, 151]/255)
hold on
plot(dt_2_3_c3, dist_2_3_c3, 'linewidth', 1.8, 'color', [254, 67, 101]/255)

% Mark the minium value
text(dt_1_2_c3(index_1_2_c3 - 35),mindist_1_2_c3 + 0.1,'o','color', [255, 94, 72]/255, 'FontSize',15)
text(dt_1_2_c3(index_1_2_c3 + 25), mindist_1_2_c3 - 0.2, ...
     ['(',num2str(dt_1_2_c3(index_1_2_c3), '%.2f'),',',num2str(mindist_1_2_c3, '%.2f'),')'], ...
     'color', 'k', 'Interpreter', 'latex', 'FontSize',8);
text(dt_1_3_c3(index_1_3_c3 - 25),mindist_1_3_c3 + 0.1,'o','color', [255, 94, 72]/255, 'FontSize',15)
text(dt_1_3_c3(index_1_3_c3 + 25), mindist_1_3_c3 - 0.2, ...
     ['(',num2str(dt_1_3_c3(index_1_3_c3), '%.2f'),',',num2str(mindist_1_3_c3, '%.2f'),')'], ...
     'color', 'k', 'Interpreter', 'latex', 'FontSize',8);
text(dt_2_3_c3(index_2_3_c3 - 25),mindist_2_3_c3 + 0.1,'o','color', [255, 94, 72]/255, 'FontSize',15)
text(dt_2_3_c3(index_2_3_c3 + 25), mindist_2_3_c3 - 0.2, ...
     ['(',num2str(dt_2_3_c3(index_2_3_c3), '%.2f'),',',num2str(mindist_2_3_c3, '%.2f'),')'], ...
     'color', 'k', 'Interpreter', 'latex', 'FontSize',8);

axis([0, 1300, 0 5])
set(gca, 'linewidth', 1.1, 'fontsize', 16)
set(gca,'TickLabelInterpreter','latex') 

%xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize',24)
ylabel('Relative Distance (m)', 'Interpreter', 'latex', 'FontSize',24)

set(gca,'xgrid','on')
set(gca,'ygrid','on')
set(gca,'Box','off')

set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'points');
%set(gcf, 'PaperPosition', [0 0 640 480]);
set(gcf, 'PaperOrientation', 'landscape');

subplot(3,1,3)
plot(dt_1_2_c4, dist_1_2_c4, 'linewidth', 1.8, 'color', [36, 169, 225]/255)
hold on
plot(dt_1_3_c4, dist_1_3_c4, 'linewidth', 1.8, 'color', [29, 191, 151]/255)
hold on
plot(dt_1_4_c4, dist_1_4_c4, 'linewidth', 1.8, 'color', [229, 131, 8]/255)
hold on
plot(dt_2_3_c4, dist_2_3_c4, 'linewidth', 1.8, 'color', [254, 67, 101]/255)
hold on
plot(dt_2_4_c4, dist_2_4_c4, 'linewidth', 1.8, 'color', [119, 52, 96]/255)
%hold on
%plot(dt_3_4_c4, dist_3_4_c4)

% Mark the minium value
text(dt_1_2_c4(index_1_2_c4 - 35),mindist_1_2_c4 + 0.1,'o','color', [255, 94, 72]/255, 'FontSize',15)
text(dt_1_2_c4(index_1_2_c4 - 900), mindist_1_2_c4 - 0.3, ...
     ['(',num2str(dt_1_2_c4(index_1_2_c4), '%.2f'),',',num2str(mindist_1_2_c4, '%.2f'),')'], ...
     'color', 'k', 'Interpreter', 'latex', 'FontSize',8);
text(dt_1_3_c4(index_1_3_c4 - 35), mindist_1_3_c4 + 0.1,'o','color', [255, 94, 72]/255, 'FontSize',15)
text(dt_1_3_c4(index_1_3_c4 + 105), mindist_1_3_c4 - 0.3, ...
     ['(',num2str(dt_1_3_c4(index_1_3_c4), '%.2f'),',',num2str(mindist_1_3_c4, '%.2f'),')'], ...
     'color', 'k', 'Interpreter', 'latex', 'FontSize',8);
text(dt_1_4_c4(index_1_4_c4 - 35),mindist_1_4_c4 + 0.1,'o','color', [255, 94, 72]/255, 'FontSize',15)
text(dt_1_4_c4(index_1_4_c4 + 75), mindist_1_4_c4, ...
     ['(',num2str(dt_1_4_c4(index_1_4_c4), '%.2f'),',',num2str(mindist_1_4_c4, '%.2f'),')'], ...
     'color', 'k', 'Interpreter', 'latex', 'FontSize',8);
text(dt_2_3_c4(index_2_3_c4 - 50),mindist_2_3_c4 + 0.2,'o','color', [255, 94, 72]/255, 'FontSize',15)
text(dt_2_3_c4(index_2_3_c4 - 25), mindist_2_3_c4 - 0.25, ...
     ['(',num2str(dt_2_3_c4(index_2_3_c4), '%.2f'),',',num2str(mindist_2_3_c4, '%.2f'),')'], ...
     'color', 'k', 'Interpreter', 'latex', 'FontSize',8);
text(dt_2_4_c4(index_2_4_c4 - 30),mindist_2_4_c4 + 0.05,'o','color', [255, 94, 72]/255, 'FontSize',15)
text(dt_2_4_c4(index_2_4_c4 - 15), mindist_2_4_c4 - 0.3, ...
     ['(',num2str(dt_2_4_c4(index_2_4_c4), '%.2f'),',',num2str(mindist_2_4_c4, '%.2f'),')'], ...
     'color', 'k', 'Interpreter', 'latex', 'FontSize',8);
%text(dt_3_4_c4(index_3_4_c4),mindist_3_4_c4,'o','color', [255, 94, 72]/255, 'FontSize',15)
%text(dt_3_4_c4(index_3_4_c4), mindist_3_4_c4, ...
%     ['(',num2str(dt_3_4_c4(index_3_4_c4), '%.2f'),',',num2str(mindist_3_4_c4, '%.2f'),')'], ...
%     'color', 'k', 'Interpreter', 'latex', 'FontSize',8);
 
axis([0, 1300, 0 5])
set(gca, 'linewidth', 1.1, 'fontsize', 16)
set(gca,'TickLabelInterpreter','latex') 

xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize',24)
%ylabel('Relative Distance (m)', 'Interpreter', 'latex', 'FontSize',24)

l1 = legend('Agent 1 \& 2', 'Agent 1 \& 3', 'Agent 1 \& 4', 'Agent 2 \& 3', 'Agent 2 \& 4')
set(l1, 'FontSize',15, 'Location','Best', 'Interpreter', 'latex')

set(gca,'xgrid','on')
set(gca,'ygrid','on')
set(gca,'Box','off')

set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'points');
%set(gcf, 'PaperPosition', [0 0 640 480]);
set(gcf, 'PaperOrientation', 'landscape');

print('-dpdf','ijrr2013-ca1_real_relative_distance.pdf', '-r400', '-bestfit')
