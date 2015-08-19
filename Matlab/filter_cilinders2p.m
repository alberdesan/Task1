clear;

NumbK=2;

sig=0.1;

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
             
        
     
P=load('Exp3EC.mat');
color=[1 0 0;0 1 0;0 0 1; 1 1 0;1 0 1; 0 1 1];
figure();
hold on;


lastk=[0,0,0,0,0,0];
newk=[0,0,0,0,0,0];
o=[0,0,0,0,0,0];
dt=0.5;
f=[0,0,0,0,0,0];
dtm=[P.V(1,3),0,0,0,0,0];
idtm=[P.V(1,3),0,0,0,0,0];
nsec=[P.V(1,2),0,0,0,0,0];
nk=[P.V(1,1),0,0,0,0,0];

F=[0 0 0 0 0 0];

H=zeros(6,NumbK);
Rad=zeros(6,NumbK);

tk=ones(6,1);


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
    
    rr=0;
    for kk=1:k-1
        tr=sqrt((PT(kk,1)-xc)*(PT(kk,1)-xc)+(PT(kk,2)-yc)*(PT(kk,2)-yc))*1.33;
        if tr>rr
            rr=tr;
        end        
    end    
    
 
    
    P.V(i,1)
    %% xc yc Filter
    F
   
    
    dtpT=P.V(i,3);
    newkT=P.V(i,1);
    
    for l=1:6
        if F(l)==1
           if ((dtpT-dtm(l))/1000)>2*dt
               F(l)=0;
               % ERASE FILTER
               Spred(l,:)=[0 0 0 0];
               Opred(l,:,:)=[0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0];
               Epred(l,:)=[0 0 0 0];
               H(l,:)=[0 0];
               Rad(l,:)=[0 0];
           end
        end
    end
    
    nf=0;
    for l=1:6
        if F(l)~=0
            nf=nf+1;
        end
    end
    
    if nf==0
        disp('Entro1')
        F(1)=1;
        % INIT
        lastk(1)=0;
        newk(1)=newkT;
        o(1)=0;
        %INITIALIZE
        disp('Init')
        % Initial values
        Spred(1,:)=[0 0 0 0];
        Opred(1,:,:)=[0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0];
        Epred(1,:)=[0 0 0 0];
        Sant(1,:) = [xc yc 0 0];
        Pant(1,:,:) = Q;
        Oant(1,:,:) = inversa(1,Pant);
        Eant(1,:) = inversa(1,Pant)*Sant(1,:)';
        
        % estimated initial position of the target
        t(1)=1;
        xx(1,t(1))=xc;
        yy(1,t(1))=yc;
        plot(xx(1,t(1)),yy(1,t(1)),'rx');
        
        theta = 0:0.05:2*pi;
        Xx = rr*cos(theta)+xx(1,t(1));
        Yy = rr*sin(theta)+yy(1,t(1));
        Yy(end) = yy(1,t(1));
        z1 = 0;
        z2 = h;
        
        patch(Xx,Yy,z2*ones(size(Xx)),'FaceColor',color(1,:),'FaceAlpha',1);
        surf([Xx;Xx],[Yy;Yy],[z1*ones(size(Xx));z2*ones(size(Xx))],'FaceColor',color(1,:),'FaceAlpha',1);
        
        
        % Initialization previous values
        for l=1:NumbK
            xant(1,l)=0;
            yant(1,l)=0;
        end
        
        
        lastk(1)=newk(1);
        IMh=1;
        
        
        trfx(1,tk(1))=xc;
        trfy(1,tk(1))=yc;
        trkt(1,tk(1))=P.V(i,3)-idtm(1);
        tk(1)=tk(1)+1;
        
        pause(0.01);
        clf;
    else
        DT=[0 0 0 0 0 0];
        ndt=0;
        for l=1:6
            if F(l)==1
                if ((dtpT-dtm(l))/1000)<dt
                    DT(l)=1;
                    ndt=ndt+1;
                end
            end
        end
        if ndt==0
            for l=1:6
                if F(l)==0
                    F(l)=1;
                    break;
                end
            end
            % INIT
            newk(l)=newkT;
            lastk(l)=0;
            o(l)=0;
            %INITIALIZE
            disp('Entro2')            
            disp('Init')
            % Initial values
            Spred(l,:)=[0 0 0 0];
            Opred(l,:,:)=[0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0];
            Epred(l,:)=[0 0 0 0];
            Sant(l,:) = [xc yc 0 0];
            Pant(l,:,:) = Q;
            Oant(l,:,:) = inversa(l,Pant);
            Eant(l,:) = inversa(l,Pant)*Sant(l,:)';
            
            % estimated initial position of the target
            t(l)=1;
            xx(l,t(l))=xc;
            yy(l,t(l))=yc;
            plot(xx(l,t(l)),yy(l,t(l)),'rx');
            
            theta = 0:0.05:2*pi;
            Xx = rr*cos(theta)+xx(l,t(l));
            Yy = rr*sin(theta)+yy(l,t(l));
            Yy(end) = yy(l,t(l));
            z1 = 0;
            z2 = h;
            
            patch(Xx,Yy,z2*ones(size(Xx)),'FaceColor',color(l,:),'FaceAlpha',1);
            surf([Xx;Xx],[Yy;Yy],[z1*ones(size(Xx));z2*ones(size(Xx))],'FaceColor',color(l,:),'FaceAlpha',1);
            
            
            % Initialization previous values
            for ll=1:NumbK
                xant(l,ll)=0;
                yant(l,ll)=0;
            end
            
            lastk(l)=newk(l);
            IMh=l;
            
            
            trfx(l,tk(l))=xc;
            trfy(l,tk(l))=yc;
            trkt(l,tk(l))=P.V(i,3)-idtm(1);
            tk(l)=tk(l)+1;
            
            pause(0.01);
            clf;
        else
            Mhh=10000;
            dMh=1;
            IMh=0;
            for l=1:6
                % Mahalanobis distance computation and association
                % selection
                
                if F(l)==1 && DT(l)==1
                    if o(l)==1
                    Mhl=([xc,yc,0,0]-[Spred(l,1),Spred(l,2),0,0])*submat(l,Opred)*([xc,yc,0,0]-[Spred(l,1),Spred(l,2),0,0])';
                    else
                    Mhl=([xc,yc,0,0]-[Sant(l,1),Sant(l,2),0,0])*submat(l,Oant)*([xc,yc,0,0]-[Sant(l,1),Sant(l,2),0,0])';                    
                    end
                    if Mhl<Mhh
                        IMh=l;
                        Mhh=Mhl;
                    end
                end
            end
            IMh
            Mhh
%             %% Filter prediction and update if there is not measure
%             for l=1:6
%                 if F(l)==1 && DT(l)==0
%                     % PREDICTION
%                     disp('Prediction')
%                     inversa(l,Oant);
%                     Opred(l,:,:) = inv(A*inversa(l,Oant)*A' + Q);
%                     Epred(l,:) = submat(l,Opred)*( A*Sant(l,:)' );
%                     Spred(l,:) = A*Sant(l,:)';
%                     O(l,:,:)=Opred(l,:,:);
%                     E(l,:)=Epred(l,:);
%                     % UPDATE
%                     disp('Update none')
%                                    
%                     % FINISH
%                     disp('Finish')
%                     disp('finNone')
%                     % Preparing next iteration
%                     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     S(l,:) = inversa(l,O)*E(l,:)';
%                     Pant(l,:,:) = inversa(l,O);
%                     Sant(l,:) = S(IMh,:);
%                     Oant(l,:,:) = O(l,:,:);
%                     Eant(l,:) = E(l,:);
%                     
%                     
%                     t(l)=t(l)+1;
%                     xx(l,t(l))=S(l,1);
%                     yy(l,t(l))=S(l,2);
%                     
%                     Mrr=0;
%                     Mh=0;
%                     
%                     H
%                     Rad
%                     
%                     for j=1:NumbK
%                         if H(l,j)~=0
%                             if H(l,j)>Mh
%                                 Mh=H(l,j);
%                             end
%                         end
%                         if Rad(l,j)~=0
%                             if Rad(l,j)>Mrr
%                                 Mrr=Rad(l,j);
%                             end
%                         end
%                     end
%                     
%                     plot(xx(l,t(l)),yy(l,t(l)),'rx');
%                     
%                     theta = 0:0.05:2*pi;
%                     Xx = Mrr*cos(theta)+xx(l,t(l));
%                     Yy = Mrr*sin(theta)+yy(l,t(l));
%                     Yy(end) = yy(l,t(l));
%                     z1 = 0;
%                     z2 = Mh;
%                     
%                     patch(Xx,Yy,z2*ones(size(Xx)),'FaceColor',color(l,:),'FaceAlpha',1);
%                     surf([Xx;Xx],[Yy;Yy],[z1*ones(size(Xx));z2*ones(size(Xx))],'FaceColor',color(l,:),'FaceAlpha',1);
%                     
%                     
%                     pause(0.01);
%                     clf;                    
%                 end
%             end

            if IMh~=0 && Mhh<dMh
                newk(IMh)=newkT;
                % Rest of the cases (For filter IMh)
                if newk(IMh)==lastk(IMh) && o(IMh)==1
                    % FINISH
                    disp('Finish')
                    disp('fin2')
                    % Preparing next iteration
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    S(IMh,:) = inversa(IMh,O)*E(IMh,:)';
                    Pant(IMh,:,:) = inversa(IMh,O);
                    Sant(IMh,:) = S(IMh,:);
                    Oant(IMh,:,:) = O(IMh,:,:);
                    Eant(IMh,:) = E(IMh,:);
                    
                    
                    t(IMh)=t(IMh)+1;
                    xx(IMh,t(IMh))=S(IMh,1);
                    yy(IMh,t(IMh))=S(IMh,2);
                    
                    Mrr=0;
                    Mh=0;
                    
                    H
                    Rad
                    
                    for j=1:NumbK
                        if H(IMh,j)~=0
                            if H(IMh,j)>Mh
                                Mh=H(IMh,j);
                            end
                        end
                        if Rad(IMh,j)~=0
                            if Rad(IMh,j)>Mrr
                                Mrr=Rad(IMh,j);
                            end
                        end
                    end
                    
                    plot(xx(IMh,t(IMh)),yy(IMh,t(IMh)),'rx');
                    
                    theta = 0:0.05:2*pi;
                    Xx = Mrr*cos(theta)+xx(IMh,t(IMh));
                    Yy = Mrr*sin(theta)+yy(IMh,t(IMh));
                    Yy(end) = yy(IMh,t(IMh));
                    z1 = 0;
                    z2 = Mh;
                    
                    patch(Xx,Yy,z2*ones(size(Xx)),'FaceColor',color(IMh,:),'FaceAlpha',1);
                    surf([Xx;Xx],[Yy;Yy],[z1*ones(size(Xx));z2*ones(size(Xx))],'FaceColor',color(IMh,:),'FaceAlpha',1);
                    

                    trfx(IMh,tk(IMh))=S(IMh,1);
                    trfy(IMh,tk(IMh))=S(IMh,2);
                    trkt(IMh,tk(IMh))=P.V(i,3)-idtm(1);
                    tk(IMh)=tk(IMh)+1;
                    
                    pause(0.01);
                    clf;
                    % PREDICTION
                    disp('Prediction')
                    inversa(IMh,Oant);
                    Opred(IMh,:,:) = inv(A*inversa(IMh,Oant)*A' + Q);
                    Epred(IMh,:) = submat(IMh,Opred)*( A*Sant(IMh,:)' );
                    Spred(IMh,:) = A*Sant(IMh,:)';
                    O(IMh,:,:)=Opred(IMh,:,:);
                    E(IMh,:)=Epred(IMh,:);
                    
                    H(IMh,:)=zeros(NumbK,1);
                    Rad(IMh,:)=zeros(NumbK,1);
                    % UPDATE
                    disp('Update')
                    [Oa(IMh,:,:),Ea(IMh,:)]=kinect(xc,yc,xant(IMh,P.V(i,1)),yant(IMh,P.V(i,1)),Spred(IMh,:)',sig);
                    
                    O(IMh,:,:)=O(IMh,:,:)+Oa(IMh,:,:);
                    E(IMh,:)=E(IMh,:)+Ea(IMh,:);
                    
                    H(IMh,P.V(i,1))=h;
                    Rad(IMh,P.V(i,1))=rr;
                    H(IMh,:)
                    Rad(IMh,:)
                end
                if newk(IMh)~=lastk(IMh) && o(IMh)==1
                    if f(IMh)==0
                        % UPDATE
                        disp('Update')
                        [Oa(IMh,:,:),Ea(IMh,:)]=kinect(xc,yc,xant(IMh,P.V(i,1)),yant(IMh,P.V(i,1)),Spred(IMh,:)',sig);
                        
                        O(IMh,:,:)=O(IMh,:,:)+Oa(IMh,:,:);
                        E(IMh,:)=E(IMh,:)+Ea(IMh,:);
                        
                        H(IMh,P.V(i,1))=h;
                        Rad(IMh,P.V(i,1))=rr;
                        H(IMh,:)
                        Rad(IMh,:)
                        
                        % FINISH
                        disp('Finish')
                        disp('fin1')
                        % Preparing next iteration
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        S(IMh,:) = inversa(IMh,O)*E(IMh,:)';
                        Pant(IMh,:,:) = inversa(IMh,O);
                        Sant(IMh,:) = S(IMh,:);
                        Oant(IMh,:,:) = O(IMh,:,:);
                        Eant(IMh,:) = E(IMh,:);
                        
                        xant(IMh,P.V(i,1))=xc;
                        yant(IMh,P.V(i,1))=yc;
                        
                        t(IMh)=t(IMh)+1;
                        xx(IMh,t(IMh))=S(IMh,1);
                        yy(IMh,t(IMh))=S(IMh,2);
                        
                        Mrr=0;
                        Mh=0;
                        
                        for j=1:NumbK
                            if H(IMh,j)~=0
                                if H(IMh,j)>Mh
                                    Mh=H(IMh,j);
                                end
                            end
                            if Rad(IMh,j)~=0
                                if Rad(IMh,j)>Mrr
                                    Mrr=Rad(IMh,j);
                                end
                            end
                        end
                        
                        plot(xx(IMh,t(IMh)),yy(IMh,t(IMh)),'rx');
                        
                        theta = 0:0.05:2*pi;
                        Xx = Mrr*cos(theta)+xx(IMh,t(IMh));
                        Yy = Mrr*sin(theta)+yy(IMh,t(IMh));
                        Yy(end) = yy(IMh,t(IMh));
                        z1 = 0;
                        z2 = Mh;
                        
                        patch(Xx,Yy,z2*ones(size(Xx)),'FaceColor',color(IMh,:),'FaceAlpha',1);
                        surf([Xx;Xx],[Yy;Yy],[z1*ones(size(Xx));z2*ones(size(Xx))],'FaceColor',color(IMh,:),'FaceAlpha',1);
                        
                        
                        lastk(IMh)=newk(IMh);
                        
                        
                        
                        trfx(IMh,tk(IMh))=S(IMh,1);
                        trfy(IMh,tk(IMh))=S(IMh,2);
                        trkt(IMh,tk(IMh))=P.V(i,3)-idtm(1);
                        tk(IMh)=tk(IMh)+1;                       
                        
                        
                        pause(0.01);
                        clf;
                        f(IMh)=1;
                    else
                        o(IMh)=1;
                        % PREDICTION
                        disp('Prediction')
                        inversa(IMh,Oant);
                        Opred(IMh,:,:) = inv(A*inversa(IMh,Oant)*A' + Q);
                        Epred(IMh,:) = submat(IMh,Opred)*( A*Sant(IMh,:)' );
                        Spred(IMh,:) = A*Sant(IMh,:)';
                        O(IMh,:,:)=Opred(IMh,:,:);
                        E(IMh,:)=Epred(IMh,:);
                        
                        H(IMh,:)=zeros(NumbK,1);
                        Rad(IMh,:)=zeros(NumbK,1);
                        
                        % UPDATE
                        disp('Update')
                        [Oa(IMh,:,:),Ea(IMh,:)]=kinect(xc,yc,xant(IMh,P.V(i,1)),yant(IMh,P.V(i,1)),Spred(IMh,:)',sig);
                        
                        O(IMh,:,:)=O(IMh,:,:)+Oa(IMh,:,:);
                        E(IMh,:)=E(IMh,:)+Ea(IMh,:);
                        
                        H(IMh,P.V(i,1))=h;
                        Rad(IMh,P.V(i,1))=rr;
                        H(IMh,:)
                        Rad(IMh,:)                        
                        
                        lastk(IMh)=newk(IMh);
                        f(IMh)=0;
                    end
                end
                lastk(IMh)
                o(IMh)
                if lastk(IMh)~=0 && o(IMh)==0
                    o(IMh)=1;
                    % PREDICTION
                    disp('Prediction')
                    inversa(IMh,Oant);
                    Opred(IMh,:,:) = inv(A*inversa(IMh,Oant)*A' + Q);
                    Epred(IMh,:) = submat(IMh,Opred)*( A*Sant(IMh,:)' );
                    Spred(IMh,:) = A*Sant(IMh,:)';
                    O(IMh,:,:)=Opred(IMh,:,:);
                    E(IMh,:)=Epred(IMh,:);
                    
                    H(IMh,:)=zeros(NumbK,1);
                    Rad(IMh,:)=zeros(NumbK,1);
                    
                    % UPDATE
                    disp('Update')
                    [Oa(IMh,:,:),Ea(IMh,:)]=kinect(xc,yc,xant(IMh,P.V(i,1)),yant(IMh,P.V(i,1)),Spred(IMh,:)',sig);
                    
                    O(IMh,:,:)=O(IMh,:,:)+Oa(IMh,:,:);
                    E(IMh,:)=E(IMh,:)+Ea(IMh,:);
                    
                    H(IMh,P.V(i,1))=h;
                    Rad(IMh,P.V(i,1))=rr;
                    H(IMh,:)
                    Rad(IMh,:)
                    
                    lastk(IMh)=newk(IMh);
                end
                if lastk(IMh)==0
                    o(IMh)=0;
                    % INIT
                    lastk(IMh)=0;
                    %INITIALIZE
                    disp('Entro3')
                    disp('Init')
                    % Initial values
                    Spred(IMh,:)=[0 0 0 0];
                    Opred(IMh,:,:)=[0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0];
                    Epred(IMh,:)=[0 0 0 0];
                    Sant(IMh,:) = [xc yc 0 0];
                    Pant(IMh,:,:) = Q;
                    Oant(IMh,:,:) = inversa(IMh,Pant);
                    Eant(IMh,:) = inversa(IMh,Pant)*Sant(IMh,:)';
                    
                    % estimated initial position of the target
                    t(IMh)=1;
                    xx(IMh,t(IMh))=xc;
                    yy(IMh,t(IMh))=yc;
                    plot(xx(IMh,t(IMh)),yy(IMh,t(IMh)),'rx');
                    
                    theta = 0:0.05:2*pi;
                    Xx = rr*cos(theta)+xx(IMh,t(IMh));
                    Yy = rr*sin(theta)+yy(IMh,t(IMh));
                    Yy(end) = yy(IMh,t(IMh));
                    z1 = 0;
                    z2 = h;
                    
                    patch(Xx,Yy,z2*ones(size(Xx)),'FaceColor',color(IMh,:),'FaceAlpha',1);
                    surf([Xx;Xx],[Yy;Yy],[z1*ones(size(Xx));z2*ones(size(Xx))],'FaceColor',color(IMh,:),'FaceAlpha',1);
                    
                    
                    % Initialization previous values
                    for ll=1:NumbK
                        xant(IMh,ll)=0;
                        yant(IMh,ll)=0;
                    end
                    
                    lastk(IMh)=newk(IMh);
                    
                    trfx(IMh,tk(IMh))=xc;
                    trfy(IMh,tk(IMh))=yc;
                    trkt(IMh,tk(IMh))=P.V(i,3)-idtm(1);
                    tk(IMh)=tk(IMh)+1;                    
                    
                    
                    pause(0.01);
                    clf;
                end
            else
                for ll=1:6
                    if F(ll)==0
                        F(ll)=1;
                        break;
                    end
                end
                % INIT
                newk(ll)=newkT;
                lastk(ll)=0;
                o(ll)=0;
                %INITIALIZE
                disp('Entro4')
                disp('Init')
                % Initial values
                Spred(ll,:)=[0 0 0 0];
                Opred(ll,:,:)=[0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0];
                Epred(ll,:)=[0 0 0 0];
                Sant(ll,:) = [xc yc 0 0];
                Pant(ll,:,:) = Q;
                Oant(ll,:,:) = inversa(ll,Pant);
                Eant(ll,:) = inversa(ll,Pant)*Sant(ll,:)';
                
                % estimated initial position of the target
                t(ll)=1;
                xx(ll,t(ll))=xc;
                yy(ll,t(ll))=yc;
                plot(xx(ll,t(ll)),yy(ll,t(ll)),'rx');
                
                theta = 0:0.05:2*pi;
                Xx = rr*cos(theta)+xx(ll,t(ll));
                Yy = rr*sin(theta)+yy(ll,t(ll));
                Yy(end) = yy(ll,t(ll));
                z1 = 0;
                z2 = h;
                
                patch(Xx,Yy,z2*ones(size(Xx)),'FaceColor',color(ll,:),'FaceAlpha',1);
                surf([Xx;Xx],[Yy;Yy],[z1*ones(size(Xx));z2*ones(size(Xx))],'FaceColor',color(ll,:),'FaceAlpha',1);
                
                
                % Initialization previous values
                for lll=1:NumbK
                    xant(ll,lll)=0;
                    yant(ll,lll)=0;
                end
                
                lastk(ll)=newk(ll);
                IMh=ll;
                
                trfx(IMh,tk(IMh))=xc;
                trfy(IMh,tk(IMh))=yc;
                trkt(IMh,tk(IMh))=P.V(i,3)-idtm(1);
                tk(IMh)=tk(IMh)+1;
                
                
                pause(0.01);
                clf;
            end
        end
    end
    
    dtm(IMh)=dtpT;
end

figure()
hold on;
for i=1:size(trkt,1)
plot(trkt(i,:)/1000,trfx(i,:),'Color',color(i,:));
end

figure()
hold on;
for i=1:size(trkt,1)
plot(trkt(i,:)/1000,trfy(i,:),'Color',color(i,:));
end
