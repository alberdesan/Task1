load('P1.txt');
color=[1 0 0;0 1 0;0 0 1; 1 1 0;1 0 1; 0 1 1];
figure();
hold on;
for i=1:length(P1)
    hold on;
    color2=color(P1(i,9),:);
    for j=10:5:130
        if (P1(i,j+1)==2)
            plot3(P1(i,j+2),P1(i,j+3),P1(i,j+4),'o','Color',color2);
        end
    end
    pause(0.1);
    clf;
end