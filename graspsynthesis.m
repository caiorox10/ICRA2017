%This software takes 4 points A,B,C,D and returns the feasible region,
%without considering the distance d between the  point fingers.
clear all 
close all
clc
%User inputs object coordinates
prompt = 'Please input figure corners [C]: ';
P1in = {input(prompt)};
prompt1 = 'Please input figure corners [D]: ';
P2in = {input(prompt1)};
prompt2 = 'Please input figure corners [B]: ';
P3in = {input(prompt2)};
prompt3 = 'Please input figure corners [A]: ';
P4in = {input(prompt3)};

P = [P1in P2in P3in P4in];
wg = 9.81;%Weight
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
%Calculate the center of mass and the angle figure makes with horizontal
T1x = (Ax + Bx + Cx)/3;
T1y = (Ay + By + Cy)/3;
T2x = (Bx + Cx + Dx)/3;
T2y = (By + Cy + Dy)/3;

xcm = (T1x+T2x)/2;
ycm = (T1y+T2y)/2;

%Calculate minimun width for fingers (Optional for software, useful for hardware implementation) 
w1 = sqrt((By-Dy)^2+(Bx-Dx)^2);
w2 = sqrt((Ay-Cy)^2+(Ax-Cx)^2);
w = min(w1,w2);

%Size of steps aling each edge
stepsize = 0.05;%Step for iterating along both sides
mu = 0.4; %Friction coeficient
good = 0; %Number of good grasps
antip = 0;
%Vary points along both edges
N = 10;
iterations = N

for theta = 0.1:0.2:1.1 %Theta angle, that also varies
%Vertice coordinates after rotation theta
Axr = cos(theta)*(Ax-Cx) - sin(theta)*(Ay-Cy) + Cx;
Ayr = sin(theta)*(Ax-Cx) + cos(theta)*(Ay-Cy) + Cy;
Bxr = cos(theta)*(Bx-Cx) - sin(theta)*(By-Cy) + Cx;
Byr = sin(theta)*(Bx-Cx) + cos(theta)*(By-Cy) + Cy;
Cxr = cos(theta)*(Cx-Cx) - sin(theta)*(Cy-Cy) + Cx;
Cyr = sin(theta)*(Cx-Cx) + cos(theta)*(Cy-Cy) + Cy;
Dxr = cos(theta)*(Dx-Cx) - sin(theta)*(Dy-Cy) + Cx;
Dyr = sin(theta)*(Dx-Cx) + cos(theta)*(Dy-Cy) + Cy;
    

L1x = linspace(Axr,Bxr,N);
L2x = linspace(Cxr,Dxr,N);
L1y = linspace(Ayr,Byr,N);
L2y = linspace(Cyr,Dyr,N);


xf1r = L1x;
yf1r = L1y;
xf2r = L2x;
yf2r = L2y;

xcmr = cos(theta)*(xcm-Cx) - sin(theta)*(ycm-Cy) + Cx;
ycmr = sin(theta)*(xcm-Cx) + cos(theta)*(ycm-Cy) + Cy;
%  i = 5;j=iterations;
      for i = 1:iterations %Loop for varying point finger 2
          for j = 1:iterations %Loop for varying point finger 1 

        %Test for antipodal grasps
        
        %Perpendicular normal
        e1p = [xf1r(j)+0.5*cos(theta) yf1r(j)+0.5*sin(theta)];
        e2p = [xf2r(i)-0.5*cos(theta) yf2r(i)-0.5*sin(theta)];
        
        ang1 = -pi/2;
        ang2 = -pi/2;
        R1 = rotation(ang1);
        R2 = rotation(ang2);

        Bn = R1*([e1p(1) e1p(2)]'-[xf1r(j) yf1r(j)]')+[xf1r(j) yf1r(j)]';
        Dn = R2*([e2p(1) e2p(2)]'-[xf2r(i) yf2r(i)]')+[xf2r(i) yf2r(i)]';
        %Normal
        e1 = [Bn(1) Bn(2)];
        e2 = [Dn(1) Dn(2)];


        pt1 = [xf1r(j) yf1r(j)];
        pt2 = [xf2r(i) yf2r(i)];

        Rconep = rotation(atan(mu));
        Rconen = rotation(-atan(mu)); 
        %Friction cones
        Cone1vecp = Rconep*([e1(1) e1(2)]'-[xf1r(j) yf1r(j)]')+[xf1r(j) yf1r(j)]';
        Cone2vecp = Rconep*([e2(1) e2(2)]'-[xf2r(i) yf2r(i)]')+[xf2r(i) yf2r(i)]';
                    Cone1p = [Cone1vecp(1) Cone1vecp(2)];
                    Cone2p = [Cone2vecp(1) Cone2vecp(2)];
        Cone1vecn = Rconen*([e1(1) e1(2)]'-[xf1r(j) yf1r(j)]')+[xf1r(j) yf1r(j)]';
        Cone2vecn = Rconen*([e2(1) e2(2)]'-[xf2r(i) yf2r(i)]')+[xf2r(i) yf2r(i)]';
                    Cone1n = [Cone1vecn(1) Cone1vecn(2)];
                    Cone2n = [Cone2vecn(1) Cone2vecn(2)];          
%         hold on
%         axis equal
%         %Plot friction cones for vizualization 
%         plot([xf1r(j) Cone1p(1)],[yf1r(j) Cone1p(2)]);
%         plot([xf1r(j) Cone1n(1)],[yf1r(j) Cone1n(2)]); 
%         plot([xf2r(i) Cone2p(1)],[yf2r(i) Cone2p(2)]);
%         plot([xf2r(i) Cone2n(1)],[yf2r(i) Cone2n(2)]); 
%         plot([e2p(1) xf2r(i)],[e2p(2) yf2r(i)],'linewidth',5)
%         plot([e1(1) xf1r(j)],[e1(2) yf1r(j)],'linewidth',5)
%         plot([e2(1) xf2r(i)],[e2(2) yf2r(i)],'linewidth',5)
%         plot([e1p(1) xf1r(j)],[e1p(2) yf1r(j)],'linewidth',5)
%         %Test antipodal grasp        
        [antipodal] = isantipodal(pt1,pt2,Cone1n,Cone1p,Cone2n,Cone2p);
        
        if antipodal == 1
            antip = antip + 1;
            F1an(antip)= xf1r(j);
            F1an(antip)= yf1r(j);
            F2an(antip)= xf2r(i);
            F2an(antip)= yf2r(i);
            thetan(antip) = theta;
            df1an = sqrt((yf1r(j)-Ayr)^2+(xf1r(j)-Axr)^2);
            df2an = sqrt((yf2r(i)-Cyr)^2+(xf2r(i)-Cxr)^2);

            f1disan(antip)=df1an;
            f2disan(antip)=df2an;
                
               
        %If not antipodal grasp, try to see if it is feasible through Linear Programming 
        else
            rx1 = xf1r(j) - xcmr;
            ry1 = yf1r(j) - ycmr;
            rx2 = xcmr - xf2r(i);
            ry2 = ycmr - yf2r(i);
            [Forces,fval,flag,contact] = isfeasible({[Axr Ayr],[Bxr Byr],[Cxr Cyr],[Dxr Dyr]},theta,[xcmr ycmr],{[rx1 ry1],[rx2 ry2]},{[xf1r(j) yf1r(j)],[xf2r(i) yf2r(i)]},mu,wg)
         
            if((flag>0)&&(contact~=1)) %successful grasp, so save the finger positions and plot
                good = good + 1;   
                F1x(good)= xf1r(j);
                F1y(good)= yf1r(j);
                F2x(good)= xf2r(i);
                F2y(good)= yf2r(i);
                thetav(good) = theta;

                df1 = sqrt((yf1r(j)-Ayr)^2+(xf1r(j)-Axr)^2);
                df2 = sqrt((yf2r(i)-Cyr)^2+(xf2r(i)-Cxr)^2);

                f1dis(good)=df1;
                f2dis(good)=df2;
               
            elseif(contact==1)
            fprintf('No configuration possible \n')
            end
        end
        end 
    end   
      end

figure
hold on

plot3(f1dis',f2dis',thetav', 'o', 'linewidth',3)
plot3(f1disan',f2disan',thetan', '+', 'linewidth',3)
PP = [f1dis' f2dis' thetav'];
k = boundary(PP);
trisurf(k,PP(:,1),PP(:,2),PP(:,3),'Facecolor','red','FaceAlpha',0.1)
PP2 = [f1disan' f2disan' thetan'];
k2 = boundary(PP2);
trisurf(k2,PP2(:,1),PP2(:,2),PP2(:,3),'Facecolor','blue','FaceAlpha',0.1)

xlabel('AB')
ylabel('CD')
zlabel('Theta')
total = antip+good;
fprintf('Stable Grasps: %d \n',total)
