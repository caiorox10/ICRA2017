clear all 
close all
clc
%User inputs object coordinates in CCW order
prompt = 'Please input figure corners: ';
Corners = (input(prompt));
n = size(Corners,2)+1;
Corners = [Corners Corners];

prompt2 = 'Please input CM: ';
CM = (input(prompt2));

for i=1:1:n+1

    V1 = Corners{i};
    V2 = Corners{i+1};
    j=i+2;
 while (j+1<=n)  
    V3 = Corners{j};
    V4 = Corners{j+1};
    P=[V1 V2 V3 V4];
    success_grasps = ngraspsynthesis(P,CM);
    j=j+1;
 end
 

end

