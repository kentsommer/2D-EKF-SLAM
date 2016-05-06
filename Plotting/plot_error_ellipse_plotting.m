function [EllipseX,EllipseY]=  plot_error_ellipse_plotting(x_hat_plus,P_plus)
gamma=3;
for i=1:1
    [eigvec,eigval]=eig(P_plus(1:2,1:2,i));
maxeigval=max(max(eigval));
mineigval=min(max(eigval));

[minindex,~]=find(eigval==mineigval);
[maxindex,~]=find(eigval==maxeigval);

mineigvec=eigvec(:,minindex);
maxeigvec=eigvec(:,maxindex);

phi=atan2(maxeigvec(2),maxeigvec(1));

phigrid = linspace(0,2*pi);

a=gamma*sqrt(maxeigval);
b=gamma*sqrt(mineigval);

x_r  = a*cos( phigrid );
y_r  = b*sin( phigrid );

ellipse=[x_r;y_r]'*[cos(phi),sin(phi);-sin(phi),cos(phi)];
ellipse(:,1);
EllipseX=ellipse(:,1) + x_hat_plus(1,i);
EllipseY=ellipse(:,2) + x_hat_plus(2,i);

%plot(ellipse(:,1) + x_hat_plus(1,i),ellipse(:,2) + x_hat_plus(2,i),'g')

end
end