clear
clc
close all

bag_dir = 'E:/IJRR2013-CA1/ijrr2013-ca1-case1&2.bag';

bot_name = 'amigobot_1';
[trajectories_1_c1] = read_trajectories_for_amigobot(bag_dir, bot_name);
bot_name = 'amigobot_2';
[trajectories_2_c1] = read_trajectories_for_amigobot(bag_dir, bot_name);
bot_name = 'amigobot_3';
[trajectories_3_c1] = read_trajectories_for_amigobot(bag_dir, bot_name);

bag_dir = 'E:/IJRR2013-CA1/ijrr2013-ca1-case3.bag';

bot_name = 'amigobot_1';
[trajectories_1_c3] = read_trajectories_for_amigobot(bag_dir, bot_name);
bot_name = 'amigobot_2';
[trajectories_2_c3] = read_trajectories_for_amigobot(bag_dir, bot_name);
bot_name = 'amigobot_3';
[trajectories_3_c3] = read_trajectories_for_amigobot(bag_dir, bot_name);

bag_dir = 'E:/IJRR2013-CA1/ijrr2013-ca1-case4.bag';

bot_name = 'amigobot_1';
[trajectories_1_c4] = read_trajectories_for_amigobot(bag_dir, bot_name);
bot_name = 'amigobot_2';
[trajectories_2_c4] = read_trajectories_for_amigobot(bag_dir, bot_name);
bot_name = 'amigobot_3';
[trajectories_3_c4] = read_trajectories_for_amigobot(bag_dir, bot_name);
bot_name = 'amigobot_4';
[trajectories_4_c4] = read_trajectories_for_amigobot(bag_dir, bot_name);

%%
figure('color',[1 1 1])
set(gcf,'outerposition',get(0,'screensize'));

subplot(1,3,1)
plot(trajectories_1_c1(:, 1), trajectories_1_c1(:, 2), 'linewidth', 2.4, 'color', [254, 67, 101]/255)
hold on
plot(trajectories_2_c1(:, 1), trajectories_2_c1(:, 2), 'linewidth', 2.4, 'color', [36, 169, 225]/255)
hold on
plot(trajectories_3_c1(:, 1), trajectories_3_c1(:, 2), 'linewidth', 2.4, 'color', [29, 191, 151]/255)

axis([-0.2, 2.5, -0.2, 4])
set(gca, 'linewidth', 1.1, 'fontsize', 16)
set(gca,'TickLabelInterpreter','latex') 

%xlabel('x (m)', 'Interpreter', 'latex', 'FontSize',24)
ylabel('y (m)', 'Interpreter', 'latex', 'FontSize',24)

set(gca,'xgrid','on')
set(gca,'ygrid','on')
set(gca,'Box','off')

subplot(1,3,2)
plot(trajectories_1_c3(:, 1), trajectories_1_c3(:, 2), 'linewidth', 2.4, 'color', [254, 67, 101]/255)
hold on
plot(trajectories_2_c3(:, 1), trajectories_2_c3(:, 2), 'linewidth', 2.4, 'color', [36, 169, 225]/255)
hold on
plot(trajectories_3_c3(:, 1), trajectories_3_c3(:, 2), 'linewidth', 2.4, 'color', [29, 191, 151]/255)

axis([-0.2, 2.5, -0.2, 4])
set(gca, 'linewidth', 1.1, 'fontsize', 16)
set(gca,'TickLabelInterpreter','latex') 

xlabel('x (m)', 'Interpreter', 'latex', 'FontSize',24)
%ylabel('y (m)', 'Interpreter', 'latex', 'FontSize',24)

set(gca,'xgrid','on')
set(gca,'ygrid','on')
set(gca,'Box','off')

subplot(1,3,3)
plot(trajectories_1_c4(:, 1), trajectories_1_c4(:, 2), 'linewidth', 2.4, 'color', [254, 67, 101]/255)
hold on
plot(trajectories_2_c4(:, 1), trajectories_2_c4(:, 2), 'linewidth', 2.4, 'color', [36, 169, 225]/255)
hold on
plot(trajectories_3_c4(:, 1), trajectories_3_c4(:, 2), 'linewidth', 2.4, 'color', [29, 191, 151]/255)
hold on
plot(trajectories_4_c4(:, 1), trajectories_4_c4(:, 2), 'linewidth', 2.4, 'color', [29, 191, 151]/255)

axis([-0.2, 2.5, -0.2, 4])
set(gca, 'linewidth', 1.1, 'fontsize', 16)
set(gca,'TickLabelInterpreter','latex') 

%xlabel('x (m)', 'Interpreter', 'latex', 'FontSize',24)
%ylabel('y (m)', 'Interpreter', 'latex', 'FontSize',24)

set(gca,'xgrid','on')
set(gca,'ygrid','on')
set(gca,'Box','off')

l1 = legend('Agent 1', 'Agent 2', 'Agent 3 \& 4')
set(l1, 'FontSize',15, 'Interpreter', 'latex')

set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'points');
%set(gcf, 'PaperPosition', [0 0 640 480]);
set(gcf, 'PaperOrientation', 'landscape');

print('-dpdf','ijrr2013-ca1_real_trajectories.pdf', '-r400', '-bestfit')
