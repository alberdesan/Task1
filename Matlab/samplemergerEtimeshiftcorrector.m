clear;
P1=load('Exp3K1.txt');
P1K=P1(1,:);
P1(1,2)=P1(1,2)-2350;
for k=3:2:length(P1)
    P1(k,2)=P1(k,2)-2350;
    P1K=[P1K;P1(k,:)];
end
P2=load('Exp3K2.txt');
O1=ones(length(P1K),1);
O2=2*ones(length(P2),1);
PP1=[O1,P1K];
PP2=[O2,P2];
P=[PP1;PP2];
[B,I]=sort(P(:,3));
V=P(I(1),:);
for i=2:length(B)
    V=[V;P(I(i),:)];
end
save('Exp3EC.mat','V');