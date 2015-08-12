
pos_cam1=[1,0,1];
rpy_cam1=[180,0,0];

r=pi*rpy_cam1(1)/180;
p=pi*rpy_cam1(2)/180;
y=pi*rpy_cam1(3)/180;

T1=[ cos(y)*cos(r)	sin(y)*sin(p)-cos(y)*sin(r)*cos(p)	sin(y)*cos(p)+cos(y)*sin(r)*sin(p)	pos_cam1(1)
     -sin(y)*cos(r)	cos(y)*sin(p)+sin(y)*sin(r)*cos(p)	cos(y)*cos(p)-sin(y)*sin(r)*sin(p)	pos_cam1(2)
            -sin(r)                          -cos(r)*cos(p)      	             cos(r)*sin(p)	pos_cam1(3)
                 0					0					0		 1 ];
             
T=[1 0 0 0
   0 1 0 0
   0 0 1 0
   0 0 0 1];

figure();
plotCamera('Location',pos_cam1,'Orientation',T1(1:3,1:3),'Opacity',0,'Size',0.05);
frame(T1,[1 0 0],0.33);
frame(T,[0 0 0],1);
axis equal;