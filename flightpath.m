%figure opens different plots
f1 = figure('Name','example1');
f2 = figure;

x=1:0.1:10;
y = sin(x);
z = x;

rotate3d on;
plot3(x,y,z)
xlabel('X');
ylabel('Y');
zlabel('Z');

figure(f1);
M = [0,0,0;1,1,1];
plotv(M);
xlabel('X2');
