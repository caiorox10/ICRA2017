function [R] = rotation(ang)
R =   [cos(ang) -sin(ang);
       sin(ang) cos(ang)];
end