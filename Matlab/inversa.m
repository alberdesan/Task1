function [A]=inversa(k,B)
A=zeros(4);
for i=1:4
    for j=1:4
        A(i,j)=B(k,i,j);
    end
end
A=inv(A);
end