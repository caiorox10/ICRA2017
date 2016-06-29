function [xint,yint] = intercept(P,d,delta)
A_ = P{4};
B_ = P{3};
C_ = P{1};
D_ = P{2};
%x coordinates of vertices
Ax = A_(1);
Bx = B_(1);
Cx = C_(1);
Dx = D_(1);
%y coordinates of vertices
Ay = A_(2);
By = B_(2);
Cy = C_(2);
Dy = D_(2);

slope = (By-Ay)/(Bx-Ax);
radius = d;
centerx = Cx+delta;
centery = Cy+delta;
y_intercept = Ay+slope*(-Ax);
[xout,yout] = linecirc(slope,y_intercept,centerx,centery,radius);

xint = max(xout);
yint = max(yout);

end