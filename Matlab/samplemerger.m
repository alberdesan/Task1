clear;
P1=load('Exp3K1.txt');
P2=load('Exp3K2.txt');
O1=ones(length(P1),1);
O2=2*ones(length(P2),1);
PP1=[O1,P1];
PP2=[O2,P2];
P=[PP1;PP2];
[B,I]=sort(P(:,3));
V=P(I(1),:);
for i=2:length(B)
    V=[V;P(I(i),:)];
end
save('Exp3.mat','V');