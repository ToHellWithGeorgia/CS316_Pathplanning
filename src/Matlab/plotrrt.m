close all; clear all;

sol = load("solution.dat");

figure; hold on;
axis([0 5 0 5 0 5]), grid on, rotate3d on;
plotcube([1,1,1],[2,2,2],.8,[0 0 1]);
plot3(sol(:,1), sol(:,2), sol(:,3), 'ro-');
xlabel('x');ylabel('y');zlabel('z');