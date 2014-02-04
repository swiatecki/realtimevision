load fpsLog.txt;
fps = fpsLog(100:end,:)


stem(fps(:,2))
%xlim([0 1000])
var(fps(:,2))
std(fps(:,2))
mean(fps(:,2))