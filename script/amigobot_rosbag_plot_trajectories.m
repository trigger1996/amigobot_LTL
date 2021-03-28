clear
clc
close all

% C:/Users/ghost/Desktop/ijrr2013-ca1/Gazebo/IJRR2013-CA1-Gazebo_Case3.bag
bag_dir = 'C:/Users/ghost/Desktop/ijrr2013-ca1/Physical/ijrr2013-ca1-case1&2.bag';

bot_name = 'amigobot_1';
[trajectories_1] = read_trajectories_for_amigobot(bag_dir, bot_name);
bot_name = 'amigobot_2';
[trajectories_2] = read_trajectories_for_amigobot(bag_dir, bot_name);
bot_name = 'amigobot_3';
[trajectories_3] = read_trajectories_for_amigobot(bag_dir, bot_name);

%%
figure('color',[1 1 1])
set(gcf,'outerposition',get(0,'screensize'));

plot(trajectories_1(:, 1), trajectories_1(:, 2), 'linewidth', 2.4, 'color', [254, 67, 101]/255)
hold on
plot(trajectories_2(:, 1), trajectories_2(:, 2), 'linewidth', 2.4, 'color', [36, 169, 225]/255)
hold on
plot(trajectories_3(:, 1), trajectories_3(:, 2), 'linewidth', 2.4, 'color', [29, 191, 151]/255)

axis([-0.2, 2.5, -0.2, 4])      % [-0.2, 2.5, -0.2, 4] [-1, 16, -20, 10]
set(gca, 'linewidth', 1.1, 'fontsize', 16)
set(gca,'TickLabelInterpreter','latex') 

xlabel('x ($m$)', 'Interpreter', 'latex', 'FontSize',24)
ylabel('y ($m$)', 'Interpreter', 'latex', 'FontSize',24)

set(gca,'xgrid','on')
set(gca,'ygrid','on')
set(gca,'Box','off')

l1 = legend('Agent 1', 'Agent 2', 'Agent 3');
set(l1, 'FontSize',15, 'Interpreter', 'latex')

set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'points');
%set(gcf, 'PaperPosition', [0 0 640 480]);
set(gcf, 'PaperOrientation', 'landscape');

print('-dpdf','ijrr2013-ca1_real_trajectories.pdf', '-r400', '-bestfit')
