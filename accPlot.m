load twoXlog.txt;
% distX, distY, x,y,z,r,p,y,a,t_min

hold on
plot(twoXlog(:,3),'*')


hold off



%%
load 2-twoXsomeYlog.txt;
hold all
plot(X2_twoXsomeYlog(:,3),'r-')
plot(X2_twoXsomeYlog(:,4),'b-')

hold off