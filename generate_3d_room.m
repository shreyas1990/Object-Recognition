% EE5369 Project
% Rohit Rawat and Shreyas Somashekar
% Object matching using Kinect

close all;
clear all;
clc;

%%%%%%%%% PART 1 CREATING THE 3D SCENE %%%%%%%%%%%%%%%%%%
figure(1)
title('Virtual 3D scene');
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
view(128,28);
% Image plane figure
figure(2)
title('Image plane of {C}')

%% a) Camera intrinsic parameters
fku=200;
fkv=200;
u0=300;
v0=300;
% Create the intrinsic camera calibration matrix "K"
K=[fku   0   u0;
    0   fkv  v0;
    0     0    1];

figure(1);
% 3D scene
hold on;
%%% initializing the normal matrices%%%
n_xz = [0 1 0]';
n_yz = [1 0 0]';
n_xy = [0 0 1]';
%% constructing the room using EGT %% 
plane_xz = f_3Dplane(n_xz,0,[0,1],[0,1],[0,1]);
plane_yz = f_3Dplane(n_yz,0,[0,1],[0,1],[0,1]);
plane_xy = f_3Dplane(n_xy,0.1,[0,0.75],[0,0.75],[0,1]);
plane_xy_floor = f_3Dplane(n_xy,0,[0,1],[0,1],[0,1]);

%% positioning the camera %%
W_t_C = [0.8 0.25 0.8]'; % W frame to C
W_R_C0 = rotox(-pi/2);  % W frame to C0
C0_R_W = W_R_C0'; % C0 to W
C0_R_C = rotoy(-pi/2-pi/4)*rotox(-pi/4); % C0 to C
W_R_C = W_R_C0*C0_R_C;  % W to C
C_R_W = W_R_C'; % C to W
H=f_Rt2H(C0_R_C,W_t_C); 
scale=1/20;
f_3Dframe(H,'r',scale*3,'_{C}'); 
f_3Dcamera(H,'r',scale,2);


%% placing the objects on the planes %%
W_Xpic1 = [0.4 0 0.5; 0.4 0 0.6; 0.6 0 0.6; 0.6 0 0.5]';
plot3(W_Xpic1(1,:), W_Xpic1(2,:), W_Xpic1(3,:), 'o');
connect_picture(W_Xpic1);

W_Xpic2 = [0 0.4 0.5; 0 0.4 0.6; 0 0.6 0.6; 0 0.6 0.5]';
plot3(W_Xpic2(1,:), W_Xpic2(2,:), W_Xpic2(3,:), 'o');
connect_picture(W_Xpic2);

W_Xbook1_unrotated = [0.3 0.1 0.1; 0.3 0.2 0.1; 0.5 0.2 0.1; 0.5 0.1 0.1]';
W_Xbook1 = rotoz(pi/4) * W_Xbook1_unrotated;
plot3(W_Xbook1(1,:), W_Xbook1(2,:), W_Xbook1(3,:), 'o');
connect_picture(W_Xbook1);

N = 100;
nn = 0.02;  %% adding noise to the scene
W_rand_pts_xz = rand(3,N);
W_rand_pts_xz(3,:) = W_rand_pts_xz(3,:)*0.9 + 0.1;
W_rand_pts_xz(2,:) = randn(1,N)*nn;
plot3(W_rand_pts_xz(1,:), W_rand_pts_xz(2,:), W_rand_pts_xz(3,:), 'r.');

W_rand_pts_yz = rand(3,N);
W_rand_pts_yz(3,:) = W_rand_pts_yz(3,:)*0.9 + 0.1;
W_rand_pts_yz(1,:) = randn(1,N)*nn;
plot3(W_rand_pts_yz(1,:), W_rand_pts_yz(2,:), W_rand_pts_yz(3,:), 'r.');

W_rand_pts_xy = 0.75*rand(3,N);
W_rand_pts_xy(3,:) = 0.1+randn(1,N)*nn;
plot3(W_rand_pts_xy(1,:), W_rand_pts_xy(2,:), W_rand_pts_xy(3,:), 'r.');

%% filling the objects %%
[a b]=meshgrid(0.4:0.01:0.6, 0.5:0.01:0.6);
W_Xpic1_fill = zeros(3,numel(a));
W_Xpic1_fill(1,:) = a(:)';
W_Xpic1_fill(3,:) = b(:)';
plot3(W_Xpic1_fill(1,:), W_Xpic1_fill(2,:), W_Xpic1_fill(3,:), 'r.');

[a b]=meshgrid(0.4:0.01:0.6, 0.5:0.01:0.6);
W_Xpic2_fill = zeros(3,numel(a));
W_Xpic2_fill(2,:) = a(:)';
W_Xpic2_fill(3,:) = b(:)';
plot3(W_Xpic2_fill(1,:), W_Xpic2_fill(2,:), W_Xpic2_fill(3,:), 'r.');

[a b]=meshgrid(0.3:0.01:0.5, 0.1:0.01:0.2);
W_Xbook1_fill = zeros(3,numel(a));
W_Xbook1_fill(1,:) = a(:)';
W_Xbook1_fill(2,:) = b(:)';
W_Xbook1_fill(3,:) = 0.1;
W_Xbook1_fill = rotoz(pi/4) * W_Xbook1_fill;
plot3(W_Xbook1_fill(1,:), W_Xbook1_fill(2,:), W_Xbook1_fill(3,:), 'r.');

axis equal;
view(128,28);

% End plotting 3D scene

objects = {W_Xpic1 W_Xpic2 W_Xbook1 W_rand_pts_xz W_rand_pts_yz W_rand_pts_xy W_Xpic1_fill W_Xpic2_fill W_Xbook1_fill};
colors = {'b+', 'b+', 'b+', 'r.', 'g.', 'b.', 'r.', 'g.', 'b.'};

figure(2);
hold on;
axis ij
for i=1:length(objects)
    U = f_perspproj(objects{i},H,K,2);
    plot(U(1,:), U(2,:), colors{i});
end
for i=1:3
    U = f_perspproj(objects{i},H,K,2);
    connect_picture(U);
end
hold off;
axis([0 u0*2 0 v0*2]); %note [0,0] in MATLAB is bottom-left, not top-left
set(gca,'ydir','reverse');

%--------------------------------------------------------------------------

%%%%%%% PART 2 - HOUGH TRANSFORM

% Input
W_X = [W_rand_pts_xz W_rand_pts_yz W_rand_pts_xy W_Xpic1_fill W_Xpic2_fill W_Xbook1_fill];
num_points = size(W_X,2);

% Hough transform will bin by normals and distance from origin.
normals = [
    1 0 0;  % yz plane / front
    0 1 0;  % xz plane / left
    0 0 1;  % xy plane / table
    1 1 0;  % 
    0 1 1;  % 
    1 0 1;  % 
    1 1 1;  % 
    ]';
distances = -0.5:0.1:1;

dim1 = size(normals,2);
dim2 = length(distances);
votes = zeros(dim1,dim2);   %voting matrix
for i=1:dim1
    for j=1:dim2
        normal = normals(:,i);
        p = distances(j);
        points_on_the_plane = 0;
        for pt = 1:num_points
            x = W_X(:,pt);

            % Check the plane equation
            error = normal'*x - p;
            if(abs(error) < 0.01)
                points_on_the_plane = points_on_the_plane+1;
            end
        end
        votes(i,j) = points_on_the_plane;
    end
end

figure(3);
imagesc(votes);
title('Hough transform for detecting planes');
xlabel('distance');
ylabel('normals');

threshold = 30;
plane_normals = [];
plane_distance_origin = [];
num_planes = 3; % pick the top 3 planes
for n=1:num_planes
    [C I] = max(votes(:));
    C = C(1);
    I = I(1);
    if(C<threshold)
        break;
    end
    [i j] = ind2sub([dim1 dim2], I);
    votes(i,j) = 0;
    plane_normals = [plane_normals normals(:,i)];   %normals
    plane_distance_origin = [plane_distance_origin; distances(j)];  %distance from the origin
end

%--------------------------------------------------------------------------
% figure(4);
% title('Rotated points');

% Find points related to each plane and mid points.
mid_points_of_planes = zeros(3,num_planes);
points_of_planes = {};
for i=1:num_planes
    normal = plane_normals(:,i);
    p = plane_distance_origin(i);
    
    points_on_the_plane = [];
    for pt = 1:num_points
        x = W_X(:,pt);
        
        % Check the plane equation
        error = normal'*x - p;
        if(abs(error) < 0.05)
            points_on_the_plane = [points_on_the_plane x];
        end
    end
    
    mid_points_of_planes(:,i) = mean(points_on_the_plane,2);
    points_of_planes{i} = points_on_the_plane;
end

%--------------------------------------------------------------------------
%%%%% PART 3 - FOR EACH PLANE, FIND ROTATIONS AND TRANSLATIONS.
colors = {'r.', 'g.', 'b.', 'y.'};

% Create a second camera {C'} at a fixed distance from the detected plane.
d = 0.5;

[Oc Xc Yc Zc] = get_camera_axes(H);

figno = 5;
for i=1:num_planes
    figure(figno);
    title('Plotting planes detected with Hough transform..');
    xlabel('x')
    ylabel('y')
    zlabel('z')
    axis equal;
    view(48,42);
    hold off;
    f_3Dplane(plane_normals(:,1),plane_distance_origin(1),[0,1],[0,1],[0,1]);
    hold on
    for j=2:num_planes
        f_3Dplane(plane_normals(:,j),plane_distance_origin(j),[0,1],[0,1],[0,1]);
    end
    
    % Find rotation required.

    normal = plane_normals(:,i)
    plane_centroid = mid_points_of_planes(:,i);
    c1_camera_location = plane_centroid + d*normal;
    line([plane_centroid(1) c1_camera_location(1)], [plane_centroid(2) c1_camera_location(2)], ...
        [plane_centroid(3) c1_camera_location(3)]);
    
    W_t_C_C1 = c1_camera_location-Oc;   % W frame to C and then to C1
    Oc1 = Oc+W_t_C_C1;
    Xc1 = Xc+W_t_C_C1;
    Yc1 = Yc+W_t_C_C1;
    Zc1 = Zc+W_t_C_C1;
    plot3([Oc1(1),Xc1(1)],[Oc1(2),Xc1(2)],[Oc1(3),Xc1(3)])
    plot3([Oc1(1),Yc1(1)],[Oc1(2),Yc1(2)],[Oc1(3),Yc1(3)])
    plot3([Oc1(1),Zc1(1)],[Oc1(2),Zc1(2)],[Oc1(3),Zc1(3)])
    plot3(Zc1(1), Zc1(2), Zc1(3), '>');
    plot3(plane_centroid(1), plane_centroid(2), plane_centroid(3), 'o');
    axislabel='old';
    text(Xc1(1),Xc1(2),Xc1(3),strcat('X',axislabel))
    text(Yc1(1),Yc1(2),Yc1(3),strcat('Y',axislabel))
    text(Zc1(1),Zc1(2),Zc1(3),strcat('Z',axislabel))
    
    W_n1 = plane_centroid - Oc1
    W_ref_normal = Zc1 - Oc1
    C_n1 = C_R_W*W_n1
    C_ref_normal = C_R_W*W_ref_normal
    C_R_C1 = find_rotation(C_n1, C_ref_normal);
    C1_R_C = C_R_C1';
    
    %%%%%%%%% PART 4 - CALCULATE HOMOGRAPHY MATRIX (UNVERIFIED)
    % Caclulate Homography matrix
    
    C_t_C1 = C_R_W * W_t_C_C1;
    
    C1_t_C = - C1_R_C * C_t_C1;
    
    n = C_R_W*normal;
    
    H = (C1_R_C - (1/d)*C1_t_C*n');
    
    C1_X = H * points_of_planes{i};
    std(C1_X')'     %if the points are in the same plane the standard deviation is zero

    W_t_C1 = c1_camera_location;
    C0_R_C1 = C0_R_C*C_R_C1;
    H1=f_Rt2H(C0_R_C1,W_t_C1);
    scale=1/20;
    f_3Dframe(H1,'r',scale*3,sprintf('_{C1%d}', i)); 
    f_3Dcamera(H1,'r',scale,2);
    view(128,28);
    
    %%%% PART 4 ALTERNATIVE
    % Do perspective projection to get orthogonal image

    figure(figno+1);
    U = f_perspproj(points_of_planes{i},H1,K,2);
    plot(U(1,:), U(2,:), colors{i});
    axis([0 u0*2 0 v0*2]); %note [0,0] in MATLAB is bottom-left, not top-left
    set(gca,'ydir','reverse');
        
    % Alternate method (not used)
    % For each of the detected points we will know the corresponding pixel
    % in the image (points on the grid). Using dilation we get a blob in
    % the shape of the wall.
    
    % We and the real image with the blob and zero out all the other
    % regions. Then we apply homographic transorm to this full image to get
    % the unskewed image.
    
    % FINAL STEP: We then do SIFT matching on this image.
    % Please see the sift matching code in the other folder.

    figno = figno+2;
end
hold off;
