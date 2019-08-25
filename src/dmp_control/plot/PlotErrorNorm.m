function  PlotErrorNorm(Time, Y_robot_data, Yr_data)

fontsize = 14;
Errs = Y_robot_data - Yr_data;

for i=1:size(Errs,2)
    Nerr(i,1) = norm(Errs(:,i));
end

figure;
plot(Time,Nerr,'Color','b','LineWidth',2)
%xlim([0 7.714])
xlabel('time(sec)','interpreter','latex','fontsize',fontsize)
ylabel('$e_t(m)$','interpreter','latex','fontsize',fontsize)

end

