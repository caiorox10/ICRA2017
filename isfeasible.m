% function [xans,fval,exitflag] = isfeasible(Pr,theta,CM,R,XF,mu,wg) %solves optimization problem for given inputs, determining if grasp is feasible
%Function that tests wether a non-antipodal grasp is a good grasp or not
%Returns a vector with the forces xans = [N1,N2,f1,f2];
%exitflag >0 that indicates a good grasp;
%contact=1 means object lost contact with object, 0 otherwise.
function [xans,fval,exitflag] = isfeasible(Pr,agnt1,agnt2,CM,R,XF,mu,wg) %solves optimization problem for given inputs, determining if grasp is feasible

rx1 = R{1}(1);%Distances for calculating moments
ry1 = R{1}(2);
rx2 = R{2}(1);
ry2 = R{2}(2);

xf1r = XF{1}(1);%Point finger coordinates
yf1r = XF{1}(2);
xf2r = XF{2}(1);
yf2r = XF{2}(2);

Axr = Pr{1}(1);%Object x Coordinates
Bxr = Pr{2}(1);
Cxr = Pr{3}(1);
Dxr = Pr{4}(1);

Ayr = Pr{1}(2);%Object y Coordinates
Byr = Pr{2}(2);
Cyr = Pr{3}(2);
Dyr = Pr{4}(2);

xcmr = CM(1);%Center of Mass Coordinates
ycmr = CM(2);

% %Matrices for Linear Programming

% if(xf1r<Cxr)
% Aeq = [sin(theta) -sin(theta) cos(theta) cos(theta)
%        -cos(theta) cos(theta) sin(theta) sin(theta)
%        (cos(theta)*abs(xf1r-Cxr) -sin(theta)*abs(yf1r-Cyr)) (cos(theta)*abs(xf2r-Cxr) + sin(theta)*abs(yf2r-Cyr)) (-cos(theta)*abs(yf1r-Cyr) - sin(theta)*abs(xf1r-Cxr)) (-cos(theta)*abs(yf2r-Cyr) +sin(theta)*abs(xf2r-Cxr))];    
% else
% Aeq = [sin(theta) -sin(theta) cos(theta) cos(theta)
%        -cos(theta) cos(theta) sin(theta) sin(theta)
%        (-cos(theta)*(xf1r-Cxr) -sin(theta)*(yf1r-Cyr)) (cos(theta)*(xf2r-Cxr) + sin(theta)*(yf2r-Cyr)) (-cos(theta)*(yf1r-Cyr) + sin(theta)*(xf1r-Cxr)) (-cos(theta)*(yf2r-Cyr) +sin(theta)*(xf2r-Cxr))];
% end

if(xf1r<Cxr)
Aeq = [sin(agnt1) -sin(agnt2) cos(agnt1) cos(agnt2)
       -cos(agnt1) cos(agnt2) sin(agnt1) sin(agnt2)
       (cos(agnt1)*abs(xf1r-Cxr) -sin(agnt1)*abs(yf1r-Cyr)) (cos(agnt2)*abs(xf2r-Cxr) + sin(agnt2)*abs(yf2r-Cyr)) (-cos(agnt1)*abs(yf1r-Cyr) - sin(agnt1)*abs(xf1r-Cxr)) (-cos(agnt2)*abs(yf2r-Cyr) +sin(agnt2)*abs(xf2r-Cxr))];    
else
Aeq = [sin(agnt1) -sin(agnt2) cos(agnt1) cos(agnt2)
       -cos(agnt1) cos(agnt2) sin(agnt1) sin(agnt2)
       (-cos(agnt1)*(xf1r-Cxr) -sin(agnt1)*(yf1r-Cyr)) (cos(agnt2)*(xf2r-Cxr) + sin(agnt2)*(yf2r-Cyr)) (-cos(agnt1)*(yf1r-Cyr) + sin(agnt1)*(xf1r-Cxr)) (-cos(agnt2)*(yf2r-Cyr) +sin(agnt2)*(xf2r-Cxr))];
end

beq = [0
       wg
      -wg*abs(xcmr-Cxr)];
   
A = [-1 0 0 0
     0 -1 0 0
     -mu 0 1 0
     -mu 0 -1 0
     0 -mu 0 1
     0 -mu 0 -1];
 
 b = [-0.001
     -0.001
      0
      0
      0
      0];
 
%Objective function identically zero for feasibility type problem
 f = [0 0 0 0];
 [xans,fval,exitflag] = linprog(f,A,b,Aeq,beq);
%  fall1 = 0;%Test if normal contact is lost
%  fall2 = 0;
%  contact = 0;
 
% % %Test if fingers are in contact with the object
% % %  if((xf1r>Bxr)||(yf1r>Byr))
% % %     fprintf('normal contac lost')
% % %     fall1 = 1;
% % %     contact = 1;
% % %  end
% % %  if((xf2r>Dxr)||(yf2r>Dyr))
% % %     fprintf('normal contac lost')
% % %     fall2 = 1;
% % %     contact = 1;
% % %  end
 %In case a good grasp,and fingers contacting object, plot itand return values
 if exitflag>0   %&&((fall1~=1)&&(fall2~=1))
    hold on
    axis equal
    plot([Cxr Dxr Bxr Axr Cxr],[Cyr Dyr Byr Ayr Cyr])
    text(Axr,Ayr,'A');
    text(Bxr,Byr,'B');
    text(Cxr,Cyr,'C');
    text(Dxr,Dyr,'D');
    plot(xf1r,yf1r,'o','linewidth',2)
    plot(xf2r,yf2r,'o','linewidth',2)
    axis equal
    plot(xcmr , ycmr,'+')
%     contact = 0;
end


end
