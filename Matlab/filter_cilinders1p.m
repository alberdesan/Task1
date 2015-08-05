clear;

NumbK=2;

sig=0.3;

pos_cam1=[1,0,1];
rpy_cam1=[180,0,0];

pos_cam2=[0,0,1];
rpy_cam2=[180,0,0];

% System covariance matrix
Q=[1 0 0 0
   0 1 0 0
   0 0 1 0
   0 0 0 1];
Q=0.1*Q;

% Evolution matrix
A=[1 0 0.1 0
   0 1 0 0.1
   0 0 1   0
   0 0 0   1];

% Camera 1 transformation matrix
r=pi*rpy_cam1(1)/180;
p=pi*rpy_cam1(2)/180;
y=pi*rpy_cam1(3)/180;

T1=[ cos(y)*cos(r)	sin(y)*sin(p)-cos(y)*sin(r)*cos(p)	sin(y)*cos(p)+cos(y)*sin(r)*sin(p)	pos_cam1(1)
     -sin(y)*cos(r)	cos(y)*sin(p)+sin(y)*sin(r)*cos(p)	cos(y)*cos(p)-sin(y)*sin(r)*sin(p)	pos_cam1(2)
            -sin(r)                          -cos(r)*cos(p)      	             cos(r)*sin(p)	pos_cam1(3)
                 0					0					0		 1 ];
             
% Camera 2 transformation matrix
r=pi*rpy_cam2(1)/180;
p=pi*rpy_cam2(2)/180;
y=pi*rpy_cam2(3)/180;

T2=[ cos(y)*cos(r)	sin(y)*sin(p)-cos(y)*sin(r)*cos(p)	sin(y)*cos(p)+cos(y)*sin(r)*sin(p)	pos_cam2(1)
     -sin(y)*cos(r)	cos(y)*sin(p)+sin(y)*sin(r)*cos(p)	cos(y)*cos(p)-sin(y)*sin(r)*sin(p)	pos_cam2(2)
            -sin(r)                          -cos(r)*cos(p)      	             cos(r)*sin(p)	pos_cam2(3)
                 0					0					0		 1 ];             
             
        
     
P=load('Exp1E.mat');
color=[1 0 0;0 1 0;0 0 1; 1 1 0;1 0 1; 0 1 1];
figure();
hold on;

lastk=0;
newk=0;
o=0;
dt=0.5;
f=0;
dtm=P.V(1,3);
idtm=P.V(1,3);
nsec=P.V(1,2);
nk=P.V(1,1);

tt=1;

for i=1:length(P.V)
    hold on;
    plotCamera('Location',pos_cam1,'Orientation',T1(1:3,1:3),'Opacity',0,'Size',0.1);
    plotCamera('Location',pos_cam2,'Orientation',T2(1:3,1:3),'Opacity',0,'Size',0.1);
    color2=color(P.V(i,11)+1,:);
    k=1;
    for j=12:5:133
        if (P.V(i,j+1)==2)
            axis equal;
            axis([ -2 3 0 3 0 3]);
            view(2);
            if P.V(i,1)==1
            PP=T1*[P.V(i,j+2),P.V(i,j+3),P.V(i,j+4),1]';
            end
            if P.V(i,1)==2
            PP=T2*[P.V(i,j+2),P.V(i,j+3),P.V(i,j+4),1]';
            end          
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
    
    tra(tt,:)=[P.V(i,1),xc,yc,P.V(i,3)-idtm];
    tt=tt+1;
    
    P.V(i,1)
    %% xc yc Filter
    dtp=P.V(i,3);
    newk=P.V(i,1);
    if ((dtp-dtm)/1000)<dt
        if newk==lastk && o==1
            % FINISH
            disp('Finish')
            % Preparing next iteration
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            S = inv(O)*E;
            Pant = inv(O);
            Sant = S;
            Oant = O;
            Eant = E;
            
            %commented in this case
            %xant(P.V(i,1))=xc;
            %yant(P.V(i,1))=yc;
            
            t=t+1;
            xx(t)=S(1);
            yy(t)=S(2);
            plot(xx(t),yy(t),'rx');
            
            tra(tt,:)=[3,S(1),S(2),P.V(i-1,3)-idtm];
            tt=tt+1;
            
            % PREDICTION
            disp('Prediction')
            inv(Oant);
            Opred = inv(A*inv(Oant)*A' + Q);
            Epred = Opred*( A*Sant );
            Spred = A*Sant;
            O=Opred;
            E=Epred;
            
            % UPDATE
            disp('Update')
            [Oa,Ea]=kinect(xc,yc,xant(P.V(i,1)),yant(P.V(i,1)),Spred,sig);
            
            O=O+Oa;
            E=E+Ea;
        end
        if newk~=lastk && o==1
            if f==0
                % UPDATE
                disp('Update')
                [Oa,Ea]=kinect(xc,yc,xant(P.V(i,1)),yant(P.V(i,1)),Spred,sig);
                
                O=O+Oa;
                E=E+Ea;
                
                % FINISH
                disp('Finish')
                % Preparing next iteration
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                S = inv(O)*E;
                Pant = inv(O);
                Sant = S;
                Oant = O;
                Eant = E;
                
                xant(P.V(i,1))=xc;
                yant(P.V(i,1))=yc;
                
                t=t+1;
                xx(t)=S(1);
                yy(t)=S(2);
                plot(xx(t),yy(t),'rx');
                
                tra(tt,:)=[3,S(1),S(2),P.V(i,3)-idtm];
                tt=tt+1;               
                
                lastk=newk;
                f=1;
            else
                % PREDICTION
                disp('Prediction')
                inv(Oant);
                Opred = inv(A*inv(Oant)*A' + Q);
                Epred = Opred*( A*Sant );
                Spred = A*Sant;
                O=Opred;
                E=Epred;
                
                % UPDATE
                disp('Update')
                [Oa,Ea]=kinect(xc,yc,xant(P.V(i,1)),yant(P.V(i,1)),Spred,sig);
                
                O=O+Oa;
                E=E+Ea;
                
                lastk=newk;
                f=0;
            end
        end
        if lastk~=0 && o==0
            o=1;
            % PREDICTION
            disp('Prediction')
            inv(Oant);
            Opred = inv(A*inv(Oant)*A' + Q);
            Epred = Opred*( A*Sant );
            Spred = A*Sant;
            O=Opred;
            E=Epred;
            
            % UPDATE
            disp('Update')
            [Oa,Ea]=kinect(xc,yc,xant(P.V(i,1)),yant(P.V(i,1)),Spred,sig);
            
            O=O+Oa;
            E=E+Ea;
            
            lastk=newk;
        end
        if lastk==0
            o=0;
            %INITIALIZE
            disp('Init')
            % Initial values
            Spred=[0 0 0 0]';
            Opred=[0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0];
            Epred=[0 0 0 0]';
            Sant = [xc yc 0 0]';
            Pant = Q;
            Oant = inv(Pant);
            Eant = inv(Pant)*Sant;
            
            % estimated initial position of the target
            t=1;
            xx(t)=xc;
            yy(t)=yc;
            plot(xx(t),yy(t),'rx');
            
            tra(tt,:)=[3,xc,yc,P.V(i,3)-idtm];
            tt=tt+1;            
            
            % Initialization previous values
            for l=1:NumbK
                xant(l)=0;
                yant(l)=0;
            end
        
            lastk=newk;
        end
    else
        lastk=0;
        o=0;
        %INITIALIZE
        disp('Init')
        % Initial values
        Spred=[0 0 0 0]';
        Opred=[0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0];
        Epred=[0 0 0 0]';
        Sant = [xc yc 0 0]';
        Pant = Q;
        Oant = inv(Pant);
        Eant = inv(Pant)*Sant;
        
        % estimated initial position of the target
        t=1;
        xx(t)=xc;
        yy(t)=yc;
        plot(xx(t),yy(t),'rx');
        
        tra(tt,:)=[3,xc,yc,P.V(i,3)-idtm];
        tt=tt+1;             
        
        % Initialization previous values
        for l=1:NumbK
        xant(l)=0;
        yant(l)=0;
        end
        
        lastk=newk;
    end
    dtm=dtp;

    
    
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
    if P.V(i,2)>nsec || P.V(i,1)~=nk 
    nsec=P.V(i,2);
    nk=P.V(i,1);
    pause(0.01);
    clf;
    end
end
figure
plot(xx)
figure
plot(yy)

figure()
hold on;
for i=1:length(tra)
    if tra(i,1)==1
       plot(tra(i,4)/1000,tra(i,2),'bx') 
    end
    if tra(i,1)==2
       plot(tra(i,4)/1000,tra(i,2),'rx') 
    end
    if tra(i,1)==3
       plot(tra(i,4)/1000,tra(i,2),'kx') 
    end     
end

figure()
hold on;
for i=1:length(tra)
    if tra(i,1)==1
       plot(tra(i,4)/1000,tra(i,3),'bx') 
    end
    if tra(i,1)==2
       plot(tra(i,4)/1000,tra(i,3),'rx') 
    end
    if tra(i,1)==3
       plot(tra(i,4)/1000,tra(i,3),'kx') 
    end     
end