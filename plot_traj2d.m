function [] = plot_traj2d(f)
% Plot the trajectory contained in f csv file
T = readmatrix(f);
r = T(:,2:3)';
figure(1)
plot(r(1,:),r(2,:),'--k', 'LineWidth',2)
grid on
hold on
end