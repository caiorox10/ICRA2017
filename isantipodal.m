function [antipodal] = isantipodal(p1,p2,Cone1n,Cone1p,Cone2n,Cone2p)
%Function to test if a grasp is antipodal or not. 
%Returns 1 if grasp is antipodal, 0 if not.

C1 = det([(Cone1n-p1);(p2-p1)]);
C2 = det([(Cone1p-p1); (p2-p1)]);
C3 = det([(Cone2n-p2);(p1-p2)]);
C4 = det([(Cone2p-p2) ; (p1-p2)]);
threshold = 0.001; %Threshold to avoid rounding errors
if((C1>threshold)&&(C2<-threshold)&&(C3>threshold)&&(C4<-threshold))
antipodal = 1;
else
antipodal = 0;
end
