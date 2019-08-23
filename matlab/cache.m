data = outputdata;

Time = data(:,1);
Y  = data(:,2:4);
Yr = data(:,5:7);
DY = data(:,8:10);
DDY = data(:,11:13);
R = data(:,14:16);
s = data(:,17);
Fdist = data(:,18:23);

save('exp3.mat','Time','Y','Yr','DY','DDY','R','s','Fdist')