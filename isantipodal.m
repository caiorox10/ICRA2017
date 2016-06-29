function [antipodal] = isantipodal(p1,p2,Cone1n,Cone1p,Cone2n,Cone2p)
%Function to test if a grasp is antipodal or not. 
%Returns 1 if grasp is antipodal, 0 if not.

C1 = det([(Cone1n-p1);(p2-p1)]);
C2 = det([(Cone1p-p1); (p2-p1)]);
C3 = det([(Cone2n-p2);(p1-p2)]);
C4 = det([(Cone2p-p2) ; (p1-p2)]);

if((C1>0)&&(C2<0)&&(C3>0)&&(C4<0))
antipodal = 1;
else
antipodal = 0;
end
