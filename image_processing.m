%IMAGE GENERATION
%reference image
pend_length = 3:0.05:7 ;
figure,plot([5,5],[pend_length(1) pend_length(end)],"Color",'black');%plotting the pendulum
hold on
hor_lim = 4:0.05:6;
plot([hor_lim(1) hor_lim(end)],[3,3],"color",'black')
hold on
plot([hor_lim(1) hor_lim(end)],[1,1],"color",'black') %plot of the horizontal boundaries
hold on
vert_lim = 1:0.05:3;
plot([4,4],[vert_lim(1) vert_lim(end)],"Color",'black')
plot([6,6],[vert_lim(1) vert_lim(end)],"Color",'black')%plot of the vertical limits
hold on
circle(4.5,0.5,0.5)
hold on
circle(5.5,0.5,0.5)%plot of the circular wheels at the base
axis([0 10 0 10])
title('zero degree disturbance (reference image)')
hold off

%56 degrees disturbance
pend_length = 3:0.05:7;
figure,plot([5,9],[pend_length(1) pend_length(end)],"Color",'black')%plotting the pendulum
hold on
hor_lim = 4:0.05:6;
plot([hor_lim(1) hor_lim(end)],[3,3],"color",'black')
hold on
plot([hor_lim(1) hor_lim(end)],[1,1],"color",'black')%plot of the horizontal boundaries
hold on
vert_lim = 1:0.05:3;
plot([4,4],[vert_lim(1) vert_lim(end)],"Color",'black')
plot([6,6],[vert_lim(1) vert_lim(end)],"Color",'black')%plot of the vertical limits
hold on
circle(4.5,0.5,0.5)
hold on
circle(5.5,0.5,0.5)%plot of the circular wheels at the base
axis([0 10 0 10])
title('positive 52 degree disturbance ')
hold off

%negative 52 degrees disturbance
pend_length = 3:0.05:7;
figure, plot([5,1],[pend_length(1) pend_length(end)],"Color",'black')
hold on

hor_lim = 4:0.05:6;
plot([hor_lim(1) hor_lim(end)],[3,3],"color",'black')%plot of the horizontal boundaries
hold on
plot([hor_lim(1) hor_lim(end)],[1,1],"color",'black')
hold on
vert_lim = 1:0.05:3;
plot([4,4],[vert_lim(1) vert_lim(end)],"Color",'black')
plot([6,6],[vert_lim(1) vert_lim(end)],"Color",'black')%plot of the vertical limits
hold on
circle(4.5,0.5,0.5)
hold on
circle(5.5,0.5,0.5)%plot of the circular wheels at the base
axis([0 10 0 10])
title('negative 52 degree disturbance ')
hold off
%other angles were also derived in a similar fashion

%% IMAGE PROCESSING
ref_img = imread("reference_img.png"); %reading the input image
%imshow(ref_img);
ref_img = rgb2gray(ref_img);%rgb to grayscale conversion

input_img = imread("pos_52_offset.png");%neg_52_offset.png");%reading the input image
%imshow(input_img);
input_img = rgb2gray(input_img);


%cropping the images to improve computation perfomance
r = centerCropWindow2d(size(ref_img),[500 578]);
ref_crop = imcrop(ref_img,r);
% imshow(ref_crop);

r1 = centerCropWindow2d(size(input_img),[500 578]);
cropped_ip = imcrop(input_img,r1);
% imshow(cropped_ip);

%detecting reference line's pixel coordinates (boundary line)
[x_up_lft,y_up_lft] = find(ref_crop(1:345,:) < 255,1,'first');
[x_lw_lft,y_lw_lft] = find(ref_crop(1:345,:) < 255,1,'last');

quad_bd = ref_crop(132:345,298); %reference line
% imshow(quad_bd)

%detecting quadrant where pendulum lies in
[x_lim,y_lim] = find(cropped_ip(1:345,:)<255,1,'first') ;

if y_lim < y_up_lft
    flag = 1 ;%negative quadrant 
elseif y_lim >= y_up_lft
    flag = 2 ; %positive quadrant
else
    disp('pendulumn outside workspace')
end

%isolating the region where the pendulum lies in based on the quadrant the
%pendulumn lies in

if flag == 2
    mapped_input = cropped_ip(132:500,298:577);
%   imshow(mapped_input)
elseif flag == 1
    mapped_input = cropped_ip(132:500,1:298);
%   imshow(mapped_input)
end

%mathematical approach to determine offset angle
 [lower_r,lower_c] = find(mapped_input(1:214,:) < 255,1,'first');%getting the leftmost limit pendulumn pixel coordinate
 [upper_r,upper_c] = find(mapped_input(1:214,:) < 255,1,'last');%getting the right extrema limit of the pendulum in pixel coordinates
if flag == 2    
    [lower_r,lower_c] = find(mapped_input(1:214,:) < 255,1,'first');
    [upper_r,upper_c] = find(mapped_input(1:214,:) < 255,1,'last');
    if lower_r == 1 && lower_c == 1 %reference line
        theta_calc = 0 ;     
    else 
        theta_calc = round(atand(abs(upper_c-lower_c)/abs(lower_r - upper_r)));%from trigonometry
    end
elseif flag == 1
    im = flip(mapped_input, 2);
    [lower_x,lower_y] = find(im(1:214,:) < 255,1,'first');%getting the leftmost limit pendulumn pixel coordinate
    [upper_x,upper_y] = find(im(1:214,:) < 255,1,'last');%getting the rightmost limit pendulum in pixel coordinate    
    theta_calc = -round(atand(abs(upper_c-lower_c)/abs(lower_r - upper_r)));
    
end

%% edge detection and hough transform
%input is the mapped input
if flag == 1
  rot_IP = flip(mapped_input, 2);%flipping the input incase pendulum is in the negative quadrant
% imshow(rot_IP)
  BW = edge(rot_IP,'canny');%canny edge detection
  
elseif flag == 2 
    BW = edge(mapped_input,'canny');
end
 
[H,Theta,Rho] = hough(BW,'RhoResolution',1,'Theta',-90:0.5:89);% H parameter space matrix

imshow(H,[],"XData",Theta,"YData",Rho,"InitialMagnification","fit")%displaying the location of the line with the most votes
xlabel("\theta"),ylabel("\rho")
axis on, axis normal,hold on
P = houghpeaks(H,5,'threshold',ceil(0.9*max(H(:)))); %extracting peaks
x = Theta(P(:,2)); %longest line corresponding value of theta
y = Rho(P(:,1));%longest line corresponding value of rho
plot(x,y,'s','color','yellow')
colormap(gca,hot);
hold off

%% Confirmation of the detected line

%confirmation of detected lines (optional)
lines = houghlines(BW, Theta,Rho,P,'FillGap',5,'MinLength',3);%extracting line segmentss
if flag == 1
    i_put = rot_IP;
else
    i_put = mapped_input; 
end

figure,imshow(i_put), hold on
max_len = 0;

for k = 1:length(lines)
    
    %plotting the detected line on the original input image
    xy = [lines(k).point1; lines(k).point2];
    plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green')
    
    %plotting the limits of the line extracted from image processing
    plot(xy(1,1),xy(1,2),'X','LineWidth',2,'Color','red')
    plot(xy(2,1),xy(2,2),'x','LineWidth',2,"Color",'red')
    title('Detected line')
    len = norm(lines(k).point1 - lines(k).point2);
    
    if(len > max_len)
    max_len = len;
    xy_long = xy;
    end
end


hold off

%% Angle of disturbance comparison
if flag == 1
    angle_displ = -round(x);
elseif flag == 2
    angle_displ = round(x);
else
    disp('error')
end

o_p = sprintf("Theoretical angle: %d .",theta_calc);
o_p2 = sprintf("Angle determined from image processing: %d ",angle_displ);

disp(o_p)  %actual calculated value of angle of disturbance
disp(o_p2) % angle of disturbance drawn from computer vision



function h = circle(x,y,r) %plotting a circle function
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit,yunit,"Color",'black');
hold off
end