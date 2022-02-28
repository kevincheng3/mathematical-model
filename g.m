% syms x1 x2 x3 x4;
% A = [-1 0 1 0; 0 -1 0 1; -0.9 0.1 -0.8 0.8]
% [U,S,V] = svd(A)
% 
% % A = [-1 0 0; 0 -1 1; -1 1 1]
% % [V,D] = eig(A)
openExample('robotics/PlanPathToAWorkspaceGoalRegionExample')