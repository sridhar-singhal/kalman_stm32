close all;
clear;
r = 20; %Radius of circle
v = 15; %Velocity of Car
w = v/r; %Angular Velocity;
dt = 0.02;
t = (0:dt:9)'; %Time steps
ay = -(v.^2./r).*sin(w.*t);
ax = -(v.^2./r).*cos(w.*t);

x_gps = -r + r*cos(w.*t);
y_gps = r*sin(w.*t);

x = zeros([length(t) 1]);
y = x;
vx = x;
vy = x;
vy(1) = v;
for i = 2:length(t)
    x(i) = x(i-1) + vx(i-1)*dt + 0.5*ax(i-1)*dt^2;
    y(i) = y(i-1) + vy(i-1)*dt + 0.5*ay(i-1)*dt^2;
    vx(i) = vx(i-1) + ax(i-1)*dt;
    vy(i) = vy(i-1) + ay(i-1)*dt;
end
%figure();
%plot(ax);
%figure();
%plot(vx);

%figure();
%plot(ay);
figure();
plot(x,y);
title("Original");
figure();
plot(x_gps,y_gps);
title("GPS");
%figure();
%plot(vy);
%figure();
%plot(x);
%figure();
%plot(y);


n_x = wgn(length(t),1,10);
%n_y = wgn
ax_n = ax + wgn(length(t),1,20);
ay_n = ay + wgn(length(t),1,20);

figure()
plot(ax_n);

x_gps_n = x_gps + wgn(length(t),1,1);
y_gps_n = y_gps + wgn(length(t),1,1);
%plot(ax_n);
%plot(ay_n);

[x_n, y_n,vx_n,vy_n] = calc_states(ax_n,ay_n,dt,v);


%figure();
%plot(ax_n);
%figure();
%plot(vx_n);

%figure();
%plot(ay);
figure();
plot(x_n,y_n);
title("Noisy IMU");

figure();
plot(x_gps_n,y_gps_n);
title("Noisy GPS");

%plot(ax_n);
%plot(n_y);



A = [1 0 dt 0;
     0 1 0 dt;
     0 0 1 0;
     0 0 0 1];

 B = [(0.5*dt^2) 0;
     0 (0.5*dt^2)
     dt 0 ;
     0 dt];
 
sig_x = 5;
sig_y = 5;
sig_vx = 2;
sig_vy = 2;
sig_ax = 20;
sig_ay = 20;

sig_senx = 2;
sig_seny = 2;

P = [sig_x 0 0 0 ;
     0 sig_y 0 0 ;
     0 0 sig_vx 0;
     0 0 0 sig_vy];


H = [1 0 0 0;
     0 1 0 0];
 
R = [sig_senx 0;
     0 sig_seny];
Q = zeros(4,4);
x0 = [0;0;0;v;];

x_k_k_1 = x0;
P_k_k_1 = P;
I = eye(4);

xf = zeros([length(ax_n) 1]);
yf = xf;
vxf = xf;
vyf = xf;
axf = xf;
ayf = xf;



for i = 2:length(ax_n)
    u_k_k_1 = [ax_n(i);ay_n(i)];
    x_k_k_1 = A*x_k_k_1 + B*u_k_k_1;
    P_k_k_1 = A*P_k_k_1*A' + Q;
    
    %Update
    z_k = [x_gps_n(i);y_gps_n(i)];
    y_k = z_k - H*x_k_k_1;
    S_k = H*P_k_k_1*H' + R;
    invS_k = inv(S_k);
    K_k = P_k_k_1*H'*invS_k;
    x_k_k_1 = x_k_k_1 + K_k*y_k;
    P_k_k_1 = (I - K_k*H)*P_k_k_1;
    
    xf(i) = x_k_k_1(1);
    yf(i) = x_k_k_1(2);
    vxf(i) = x_k_k_1(3);
    vyf(i) = x_k_k_1(4);

end

figure();
plot(xf,yf,"r",'LineWidth',3);
hold on;
plot(x_n,y_n,"b--",'LineWidth',1);
hold on;
plot(x_gps,y_gps,"g--",'LineWidth',1);
hold on;
scatter(x_gps_n,y_gps_n);
hold off;
title("Kalmann");
legend("Filtered","Noisy IMU","Target","Noisy GPS");

function [x,y,vx,vy] = calc_states(ax,ay,dt,v)
   
x = zeros([length(ax) 1]);
y = x;
vx = x;
vy = x;
vy(1) = v;
for i = 2:length(ax)
    x(i) = x(i-1) + vx(i-1)*dt + 0.5*ax(i-1)*dt^2;
    y(i) = y(i-1) + vy(i-1)*dt + 0.5*ay(i-1)*dt^2;
    vx(i) = vx(i-1) + ax(i-1)*dt;
    vy(i) = vy(i-1) + ay(i-1)*dt;
end 
end