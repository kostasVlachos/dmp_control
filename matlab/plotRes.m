clear all
load('exp3.mat')


e=R-Y;
er=Y-Yr;
et=e+er;

plot3(R(:,1),R(:,2),R(:,3))
hold on
plot3(Yr(:,1),Yr(:,2),Yr(:,3))
hold off
xlabel('x')
ylabel('y')
zlabel('z')

for i = 1 : size(et,1)
    Normet(i) = norm(et(i,1:3));
end

figure;
plot(Time,Normet)
xlabel('time')
ylabel('|e|')

figure;
subplot(3,1,1)
plot(Time,R(:,1))
hold on
plot(Time,Yr(:,1))
hold off
xlabel('time')
ylabel('x')
subplot(3,1,2)
plot(Time,R(:,2))
hold on
plot(Time,Yr(:,2))
hold off
xlabel('time')
ylabel('y')
subplot(3,1,3)
plot(Time,R(:,3))
hold on
plot(Time,Yr(:,3))
hold off
xlabel('time')
ylabel('z')

figure;
subplot(3,1,1)
plot(Time,e(:,1))
hold on
plot(Time,er(:,1))
plot(Time,et(:,1))
hold off
xlabel('time')
ylabel('x')
l=legend('p - y','y-y_d','p-y_d');
set(l)
subplot(3,1,2)
plot(Time,e(:,2))
hold on
plot(Time,er(:,2))
plot(Time,et(:,2))
hold off
xlabel('time')
ylabel('y')
subplot(3,1,3)
plot(Time,e(:,3))
hold on
plot(Time,er(:,3))
plot(Time,et(:,3))
hold off
xlabel('time')
ylabel('z')






