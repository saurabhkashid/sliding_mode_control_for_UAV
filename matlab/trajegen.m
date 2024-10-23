
clc 
clear
close all

syms t

p0 = [0 0 0];
p1 = [0 0 1];
p2 = [1 0 1];
p3 = [1 1 1];
p4 = [0 1 1];
p5 = [0 0 1];
t0 = 0;
t1 = 5;
t2 = 15;
t2_ = 20;
t3_ = 35;
t4_ = 50;
t5_ = 65;
a1 = traject_gen(p0, p1, t0, t1);
a1_vel = jacobian(a1,t);
a1_acc = jacobian(a1_vel,t);

a2 = traject_gen(p1, p2, t1, t2_);
a2_vel = jacobian(a2,t);
a2_acc = jacobian(a2_vel,t);

a3 = traject_gen(p2, p3, t2_, t3_);
a3_vel = jacobian(a3,t);
a3_acc = jacobian(a3_vel,t);

a4 = traject_gen(p3, p4, t3_, t4_);
a4_vel = jacobian(a4,t);
a4_acc = jacobian(a4_vel,t);

a5 = traject_gen(p4, p5, t4_, t5_);
a5_vel = jacobian(a5,t);
a5_acc = jacobian(a5_vel,t);

a1_traj =[]; a2_traj =[]; a3_traj = []; a4_traj = []; a5_traj = [];
a1_dvel = []; a2_dvel = []; a3_dvel = []; a4_dvel = []; a5_dvel = [];
a1_dacc = []; a2_dacc = []; a3_dacc = []; a4_dacc = []; a5_dacc = [];
time= 5;
for i=0:0.5:time
    b = subs(a1,t,i);
    c = double(b);
    a1_traj=[a1_traj;b];
    x = subs(a1_vel,t,i);
    x = double(x);
    a1_dvel = [a1_dvel,x];
    y = subs(a1_acc,t,i);
    y = double(y);
    a1_dacc = [a1_dacc, y];
    
end

time2=15;
for x=5:0.5:20
    b1 = subs(a2,t,x);
    b1 = double(b1);
    a2_traj=[a2_traj;b1];
    x1 = subs(a2_vel,t,x);
    x1 = double(x1);
    a2_dvel = [a2_dvel,x1];
    y1 = subs(a2_acc,t,x);
    y1 = double(y1);
    a2_dacc = [a2_dacc, y1];
end
for x=20:0.5:35
    b2 = subs(a3,t,x);
    b2 = double(b2);
    a3_traj=[a3_traj;b2];
    x2 = subs(a3_vel,t,x);
    x2 = double(x2);
    a3_dvel = [a3_dvel,x2];
    y2 = subs(a3_acc,t,x);
    y2 = double(y2);
    a3_dacc = [a3_dacc, y2];
end
for x = 35:0.5:50
    b3 = subs(a4,t,x);
    b3 = double(b3);
    a4_traj=[a4_traj;b3];
    x3 = subs(a4_vel,t,x);
    x3 = double(x3);
    a4_dvel = [a4_dvel,x3];
    y3 = subs(a4_acc,t,x);
    y3 = double(y3);
    a4_dacc = [a4_dacc, y3];
end
for x = 50:0.5:65
    b4 = subs(a5,t,x);
    b4 = double(b4);
    a5_traj=[a5_traj;b4];
    x4 = subs(a5_vel,t,x);
    x4 = double(x4);
    a5_dvel = [a5_dvel,x4];
    y4 = subs(a5_acc,t,x);
    y4 = double(y4);
    a5_dacc = [a5_dacc, y4];
end

figure(1)
plot3(a1_traj(:,1),a1_traj(:,2),a1_traj(:,3))
hold on
plot3(a2_traj(:,1),a2_traj(:,2),a2_traj(:,3));
hold on
plot3(a3_traj(:,1),a3_traj(:,2),a3_traj(:,3));
hold on
plot3(a4_traj(:,1),a4_traj(:,2),a4_traj(:,3));
hold on
plot3(a5_traj(:,1),a5_traj(:,2),a5_traj(:,3));
xlabel("x-axis")
ylabel("y-axis")
zlabel("z-axis")
legend({'Traj-1','Traj-2','traj-3','traj-4','traj-5'},'Location','bestoutside')
hold off

t = 0:0.5:65;
x_traj = cat(2,a1_traj(:,1)',a2_traj(2:end,1)',a3_traj(2:end,1)',a4_traj(2:end,1)',a5_traj(2:end,1)');
y_traj = cat(2,a1_traj(:,2)',a2_traj(2:end,2)',a3_traj(2:end,2)',a4_traj(2:end,2)',a5_traj(2:end,2)');
z_traj = cat(2,a1_traj(:,3)',a2_traj(2:end,3)',a3_traj(2:end,3)',a4_traj(2:end,3)',a5_traj(2:end,3)');

x_dvel = cat(2,a1_dvel(1,:),a2_dvel(1,2:end),a3_dvel(1,2:end),a4_dvel(1,2:end),a5_dvel(1,2:end));
y_dvel = cat(2,a1_dvel(2,:),a2_dvel(2,2:end),a3_dvel(2,2:end),a4_dvel(2,2:end),a5_dvel(2,2:end));
z_dvel = cat(2,a1_dvel(3,:),a2_dvel(3,2:end),a3_dvel(3,2:end),a4_dvel(3,2:end),a5_dvel(3,2:end));

x_dacc = cat(2,a1_dacc(1,:),a2_dacc(1,2:end),a3_dacc(1,2:end),a4_dacc(1,2:end),a5_dacc(1,2:end));
y_dacc = cat(2,a1_dacc(2,:),a2_dacc(2,2:end),a3_dacc(2,2:end),a4_dacc(2,2:end),a5_dacc(2,2:end));
z_dacc = cat(2,a1_dacc(3,:),a2_dacc(3,2:end),a3_dacc(3,2:end),a4_dacc(3,2:end),a5_dacc(3,2:end));


figure(2)
plot(t,x_traj)
hold on
plot(t,y_traj)
hold on
plot(t, z_traj)
ylim([-0.5,1.5])
xlabel("Time")
ylabel("value")
legend({'X-Trajectory','Y-Trajectory','Z-Trajectory'})
hold off
% figure(2)
% plot(t,x_traj)
% hold on
% plot(t,y_traj)
% hold on
% plot(t, z_traj)
% ylim([-0.5,1.5])
% xlabel("Time")
% ylabel("value")
% legend({'X-Trajectory','Y-Trajectory','Z-Trajectory'})
% hold off

figure(3)
subplot(3,1,1)
plot(t,x_dvel )%,t,y_dvel,t,z_dvel)
title("X-desired velocity")
xlabel("Time")
ylabel("x-velocity")
subplot(3,1,2)
plot(t, y_dvel)
xlabel("Time")
ylabel("y-velocity")
title("Y-desired velocity")
subplot(3,1,3)
plot(t,z_dvel)
xlabel("Time")
ylabel("y-velocity")
title("Z-desired velocity")

figure(4)
subplot(3,1,1)
plot(t,x_dacc )%,t,y_dvel,t,z_dvel)
title("X-desired Acceleration")
xlabel("Time")
ylabel("x-acceleration")
subplot(3,1,2)
plot(t, y_dacc)
xlabel("Time")
ylabel("y-acceleration")
title("Y-desired Acceleration")
subplot(3,1,3)
plot(t,z_dacc)
xlabel("Time")
ylabel("z-acceleration")
title("Z-desired Acceleration")




function a = traject_gen(pin, pf, t0, tf)
    syms t
    A = [1, t0, t0^2, t0^3, t0^4, t0^5;
        0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;
        0, 0, 2, 6*t0, 12*t0^2, 20*t0^3;
        1, tf, tf^2, tf^3, tf^4, tf^5;
        0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
        0, 0, 2, 6*tf, 12*tf^2, 20*tf^3;];
    
    b= A\[[pin(1); 0; 0; pf(1); 0; 0],[pin(2); 0; 0; pf(2); 0; 0], [pin(3); 0; 0; pf(3); 0; 0]];
    c = [1, t, t^2, t^3, t^4, t^5];
    a = c*b; 
end