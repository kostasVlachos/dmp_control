function  PlotFdist( T, F )

fontsize=14;
figure;
plot(T,F(1,:),'Color','b','LineWidth',2)
hold on
plot(T,F(2,:),'Color','r','LineWidth',2)
plot(T,F(3,:),'Color','k','LineWidth',2)
%xlim([0 7.714])
xlabel('time(sec)','interpreter','latex','fontsize',fontsize)
ylabel('$F_{dist} (m)$','interpreter','latex','fontsize',fontsize)
hold off

end

