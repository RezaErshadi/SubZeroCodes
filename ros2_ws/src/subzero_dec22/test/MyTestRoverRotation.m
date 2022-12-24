clear
close all
clc
%%
r_pose = [0;0];
r_hdng = 0;
t0 = 0;
t1 = 5;
v = 1;
w = 0;
dt = 0.1;

T = t0:dt:t1;
figure,
for i = T
   r_pose = r_pose + (dt * v * r_hdng);
   plot(T(i),r_pose(i))
   hold on
   drawnow
end