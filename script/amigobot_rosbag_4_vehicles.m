clear
clc
close all

%C:/Users/ghost/Desktop/ijrr2013-ca1/Gazebo/IJRR2013-CA1-Gazebo_Case4.bag
%C:/Users/ghost/Desktop/ijrr2013-ca1/Physical/ijrr2013-ca1-case4.bag
bag_dir = 'C:/Users/ghost/Desktop/ijrr2013-ca1/Physical/ijrr2013-ca1-case4.bag'; % C:/Users/ghost/Desktop/ijrr2013-ca1/Gazebo/IJRR2013-CA1-Gazebo_Case1_4.bag
%bag_dir = 'E:/IJRR2013-CA1/ijrr2013-ca1-case3.bag';

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

[mindist_1_2, index_1_2] = min(dist_1_2);
[mindist_1_3, index_1_3] = min(dist_1_3);
[mindist_1_4, index_1_4] = min(dist_1_4);
[mindist_2_3, index_2_3] = min(dist_2_3);
[mindist_2_4, index_2_4] = min(dist_2_4);
[mindist_3_4, index_3_4] = min(dist_3_4);



%%
% https://zhuanlan.zhihu.com/p/77628238
% https://www.icoa.cn/a/512.html
% https://jingyan.baidu.com/article/219f4bf70445539e452d3857.html
% http://blog.sina.com.cn/s/blog_7db803c10102weyk.html
% https://www.pianshen.com/article/84681707738/

figure('color',[1 1 1])
set(gcf,'outerposition',get(0,'screensize'));

plot(dt_1_2, dist_1_2, 'linewidth', 1.8, 'color', [36, 169, 225]/255)
hold on
plot(dt_1_3, dist_1_3, 'linewidth', 1.8, 'color', [29, 191, 151]/255)
hold on
plot(dt_1_4, dist_1_4, 'linewidth', 1.8, 'color', [229, 131, 8]/255)
hold on
plot(dt_2_3, dist_2_3, 'linewidth', 1.8, 'color', [254, 67, 101]/255)
hold on
plot(dt_2_4, dist_2_4, 'linewidth', 1.8, 'color', [119, 52, 96]/255)
hold on
plot(dt_3_4, dist_3_4, 'linewidth', 1.8, 'color', [200, 200, 169]/255)

% Mark the minium value
text(dt_1_2(index_1_2 - 15),mindist_1_2 + 0.01,'o','color', [255, 94, 72]/255, 'FontSize',15)
text(dt_1_2(index_1_2 - 325), mindist_1_2 - 0.1, ...
     ['(',num2str(dt_1_2(index_1_2), '%.2f'),',',num2str(mindist_1_2, '%.2f'),')'], ...
     'color', 'k', 'Interpreter', 'latex', 'FontSize',10);
text(dt_1_3(index_1_3 - 25), mindist_1_3 + 0.01,'o','color', [255, 94, 72]/255, 'FontSize',15)
text(dt_1_3(index_1_3 + 105), mindist_1_3 - 0.1, ...
     ['(',num2str(dt_1_3(index_1_3), '%.2f'),',',num2str(mindist_1_3, '%.2f'),')'], ...
     'color', 'k', 'Interpreter', 'latex', 'FontSize',10);
text(dt_1_4(index_1_4 - 25),mindist_1_4 + 0.01,'o','color', [255, 94, 72]/255, 'FontSize',15)
text(dt_1_4(index_1_4 + 75), mindist_1_4, ...
     ['(',num2str(dt_1_4(index_1_4), '%.2f'),',',num2str(mindist_1_4, '%.2f'),')'], ...
     'color', 'k', 'Interpreter', 'latex', 'FontSize',10);
text(dt_2_3(index_2_3 - 15),mindist_2_3 + 0.01,'o','color', [255, 94, 72]/255, 'FontSize',15)
text(dt_2_3(index_2_3 - 25), mindist_2_3 - 0.1, ...
     ['(',num2str(dt_2_3(index_2_3), '%.2f'),',',num2str(mindist_2_3, '%.2f'),')'], ...
     'color', 'k', 'Interpreter', 'latex', 'FontSize',10);
text(dt_2_4(index_2_4 - 5),mindist_2_4 + 0.02,'o','color', [255, 94, 72]/255, 'FontSize',15)
text(dt_2_4(index_2_4 - 15), mindist_2_4 - 0.1, ...
     ['(',num2str(dt_2_4(index_2_4), '%.2f'),',',num2str(mindist_2_4, '%.2f'),')'], ...
     'color', 'k', 'Interpreter', 'latex', 'FontSize',10);
text(dt_3_4(index_3_4 - 20),mindist_3_4 + 0.05,'o','color', [255, 94, 72]/255, 'FontSize',15)
text(dt_3_4(index_3_4 - 15), mindist_3_4 - 0.1, ...
     ['(',num2str(dt_3_4(index_3_4), '%.2f'),',',num2str(mindist_3_4, '%.2f'),')'], ...
     'color', 'k', 'Interpreter', 'latex', 'FontSize',10);
 
axis([0, 650, 0 5])    % [0, 650, 0 35]
set(gca, 'linewidth', 1.1, 'fontsize', 16)
set(gca,'TickLabelInterpreter','latex') 

xlabel('Time ($s$)', 'Interpreter', 'latex', 'FontSize',24)
ylabel('Relative Distance ($m$)', 'Interpreter', 'latex', 'FontSize',24)

l1 = legend('Agent 1 \& 2', 'Agent 1 \& 3', 'Agent 1 \& 4', 'Agent 2 \& 3', 'Agent 2 \& 4', 'Agent 3 \& 4')
set(l1, 'FontSize',16, 'Interpreter', 'latex')      % 'Location','Best',

set(gca,'xgrid','on') %添加分割x轴的坐标网线
set(gca,'ygrid','on') %添加分割y轴的坐标网线
set(gca,'Box','off')

set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'points');
%set(gcf, 'PaperPosition', [0 0 640 480]);
set(gcf, 'PaperOrientation', 'landscape');
print('-dpdf','ijrr2013-ca1-case3.pdf', '-r400', '-bestfit')

