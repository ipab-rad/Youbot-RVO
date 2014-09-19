mu = [0 0];
Sigma = [0.25 0; 0 1];
X = -3:.1:3; Y = -3:.1:3;
[X1,X2] = meshgrid(X,Y);
F = mvnpdf([X1(:) X2(:)],mu,Sigma);
F = reshape(F,length(Y),length(X));
surf(X,Y,F);
caxis([min(F(:))-.5*range(F(:)),max(F(:))]);
axis([-3 3 -3 3 0 .4])
xlabel('x'); ylabel('y'); zlabel('Probability Density');