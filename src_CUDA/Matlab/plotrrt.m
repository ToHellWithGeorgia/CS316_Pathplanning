close all; clear all;

% sol = load("solution_single.dat");
% 
% figure; hold on;
% axis([0 5 0 5 0 5]), grid on, rotate3d on;
% plotcube([1,1,1],[2,2,2],.8,[0 0 1]);
% plot3(sol(:,1), sol(:,2), sol(:,3), 'ro-');
% xlabel('x');ylabel('y');zlabel('z');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sol = load("solution_8cube.dat");
% 
% figure; hold on;
% axis([0 5 0 5 0 5]), grid on, rotate3d on;
% plotcube([1,1,1],[1,1,1],.8,[0 0 1]);
% plotcube([1,1,1],[1,1,3],.8,[0 0 1]);
% plotcube([1,1,1],[1,3,1],.8,[0 0 1]);
% plotcube([1,1,1],[1,3,3],.8,[0 0 1]);
% plotcube([1,1,1],[3,1,1],.8,[0 0 1]);
% plotcube([1,1,1],[3,1,3],.8,[0 0 1]);
% plotcube([1,1,1],[3,3,1],.8,[0 0 1]);
% plotcube([1,1,1],[3,3,3],.8,[0 0 1]);
% plot3(sol(:,1), sol(:,2), sol(:,3), 'ro-');
% xlabel('x');ylabel('y');zlabel('z');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sol = load("solution_simple_maze.dat");

figure; hold on;
axis([0 5 0 5 0 5]), grid on, rotate3d on;
% plotcube([5,5,5],[0,0,0],.4,[0 1 0]);
plotcube([1,5,4],[1,0,0],.8,[0 0 1]);
plotcube([1,5,4],[3,0,1],.8,[0 0 1]);
plot3(sol(:,1), sol(:,2), sol(:,3), 'ro-');
xlabel('x');ylabel('y');zlabel('z');