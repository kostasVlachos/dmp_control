function  plotPath3d(Y_robot ,Yr )

fontsize=14;
figure;
plot3(Y_robot(1,:),Y_robot(2,:),Y_robot(3,:),'Color','b','LineWidth',1)
hold on
plot3(Yr(1,:),Yr(2,:),Yr(3,:),'Color','r','LineWidth',1)

scatter3(Y_robot(1,1),Y_robot(2,1),Y_robot(3,1),'MarkerEdgeColor','k',...
    'MarkerFaceColor','g')
scatter3(Yr(1,size(Yr,2)),Y_r(2,size(Yr,2)),Y_r(3,size(Yr,2)),'MarkerEdgeColor','k',...
    'MarkerFaceColor','r')

grid
xlabel('x','interpreter','latex','fontsize',fontsize)
ylabel('y','interpreter','latex','fontsize',fontsize)
zlabel('z','interpreter','latex','fontsize',fontsize)
l=legend('$y_{robot}$','$y_r$','start','goal');
set(l,'interpreter','latex','fontsize',12)
hold off;
end

