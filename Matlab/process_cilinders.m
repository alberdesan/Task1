clear;
pos_cam1=[0,0,1.5];
rpy_cam1=[180,0,0];

% Camera 1 transformation matrix
r=pi*rpy_cam1(1)/180;
p=pi*rpy_cam1(2)/180;
y=pi*rpy_cam1(3)/180;

T1=[ cos(y)*cos(r)	sin(y)*sin(p)-cos(y)*sin(r)*cos(p)	sin(y)*cos(p)+cos(y)*sin(r)*sin(p)	pos_cam1(1)
     -sin(y)*cos(r)	cos(y)*sin(p)+sin(y)*sin(r)*cos(p)	cos(y)*cos(p)-sin(y)*sin(r)*sin(p)	pos_cam1(2)
            -sin(r)                          -cos(r)*cos(p)      	             cos(r)*sin(p)	pos_cam1(3)
                 0					0					0		 1 ];
             
        
     
P=load('P4.txt');
color=[1 0 0;0 1 0;0 0 1; 1 1 0;1 0 1; 0 1 1];
figure();
hold on;

nsec=P(1,1);
for i=1:length(P)
    hold on;
    plotCamera('Location',pos_cam1,'Orientation',T1(1:3,1:3),'Opacity',0,'Size',0.1);    
    color2=color(P(i,10)+1,:);
    k=1;
    for j=11:5:131
        if (P(i,j+1)==2)
            axis equal;
            axis([ -2 2 0 2 0 2]);
            view(3);             
            PP=T1*[P(i,j+2),P(i,j+3),P(i,j+4),1]';
            plot3(PP(1),PP(2),PP(3),'o','Color',color2);            
            PT(k,:)=PP;
            k=k+1;
        end
    end
    % Cilindrization of human figures
    h=0;
    dx=0;
    dy=0;
    for kk=1:k-1
        if PT(kk,3)>h
            h=PT(kk,3);
        end
    end

    xc=mean(PT(1:k-1,1));
    yc=mean(PT(1:k-1,2));
    
    rr=0;
    for kk=1:k-1
        tr=sqrt((PT(kk,1)-xc)*(PT(kk,1)-xc)+(PT(kk,2)-yc)*(PT(kk,2)-yc));
        if tr>rr
            rr=tr;
        end        
    end
    
    theta = 0:0.05:2*pi;
    Xx = rr*cos(theta)+xc;
    Yy = rr*sin(theta)+yc;
    Yy(end) = yc;
    z1 = 0;
    z2 = h;
    
    patch(Xx,Yy,z2*ones(size(Xx)),'FaceColor',color2,'FaceAlpha',1);
    surf([Xx;Xx],[Yy;Yy],[z1*ones(size(Xx));z2*ones(size(Xx))],'FaceColor',color2,'FaceAlpha',1);
    if P(i,1)>nsec
    nsec=P(i,1);
    pause(0.05);
    clf;
    end
end