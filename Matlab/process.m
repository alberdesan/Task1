P=load('P4.txt');
color=[1 0 0;0 1 0;0 0 1; 1 1 0;1 0 1; 0 1 1];
figure();
hold on;
nsec=P(1,1);
for i=1:length(P)
    hold on;
    color2=color(P(i,10)+1,:);
    for j=11:5:131
        if (P(i,j+1)==2)
            axis([ -1 1 0 5 -1 1]);
            view(3);
            plot3(P(i,j+2),P(i,j+4),P(i,j+3),'o','Color',color2);
        end
    end
    if P(i,1)>nsec
    nsec=P(i,1);
    pause(0.01);
    clf;
    end
end