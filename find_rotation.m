function Rotation = find_rotation(normal, ref_normal)
% This function calculates the rotation matrix required to align the 
% vector normal to the vector ref_normal. The vector ref_normal is assumed 
% to be the z axis of the frame under consideration.

% find projections on the xz and yz planes
proj_xz = normal;
proj_xz(2) = 0;

proj_yz = normal;
proj_yz(1) = 0;

% Find the angle between projections and the ref normal
eps1 = 0.01;
if(norm(proj_xz) > eps1 && norm(ref_normal) > eps1)
    y_rot = acos(proj_xz'*ref_normal / (norm(proj_xz)*norm(ref_normal)));
else
    y_rot = 0;
end
if(norm(proj_yz) > eps1 && norm(ref_normal) > eps1)
    x_rot = acos(proj_yz'*ref_normal / (norm(proj_yz)*norm(ref_normal)));
else
    x_rot = 0;
end

% select sign
xz_cross_z = cross(proj_xz,ref_normal)
if(xz_cross_z(2)<0)
    y_rot = -y_rot;
end
yz_cross_z = cross(proj_yz,ref_normal)
if(yz_cross_z(1)<0)
    x_rot = -x_rot;
end

if(abs(x_rot)>pi/2)
    y_rot = pi-y_rot;
end

% eps1 = 0.01;
% if(norm(proj_xz) > eps1 && norm(ref_normal) > eps1)
%     y_rot = asin(norm(cross(proj_xz,ref_normal)) / (norm(proj_xz)*norm(ref_normal)));
% else
%     y_rot = 0;
% end
% if(norm(proj_yz) > eps1 && norm(ref_normal) > eps1)
%     x_rot = asin(norm(cross(proj_yz,ref_normal)) / (norm(proj_yz)*norm(ref_normal)));
% else
%     x_rot = 0;
% end

fprintf('Detected rotations: rotox=%f rotoy=%f\n', x_rot, y_rot);

% Return rotation matrix
Rotation = (rotoy(y_rot)*rotox(x_rot))';
