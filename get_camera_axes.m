function [Oc1 Xc1 Yc1 Zc1] = get_camera_axes(H)
% Returns the x,y,z axis of a camera specified by the given H matrix.

R=H([1:3],[1:3]);% = Rw2i
t=H([1:3],4);

scale = 0.25;

%Camera reference frame 
Oc = scale*[0,0,0]';
Xc = scale*[1,0,0]';
Yc = scale*[0,1,0]';
Zc = scale*[0,0,1]';

Ri2w=R;
Oc1=Ri2w*Oc+t;
Xc1=Ri2w*Xc+t;
Yc1=Ri2w*Yc+t;
Zc1=Ri2w*Zc+t;
