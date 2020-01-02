
format compact ; clear; close all ; clc; 

% % PUBLISHER SETUP :: FROM MATLAB TO ROS
% global pub_type msg_pub % for easy use inside of nested functions
% pub_type = rospublisher('chatter_from_matlab3','std_msgs/Float64MultiArray'); %test message   
% msg_pub  = rosmessage( pub_type ) ;

% SUBSCRIBER SETUP :: FROM ROS TO MATLAB 
global sub_type msg_sub % for easy use inside of nested functions
sub_type = rossubscriber('chatter_from_python_multi','std_msgs/Float64MultiArray'); % needs to return large 2d matrix
msg_sub  = rosmessage( sub_type ) ; 

% x1 = [ 5 0 ;
%        1  1;
%        65 89]' ; 
% x2 = [2  1;
%       3  5;
%       6 8]' ;
%   
% x = mat2vec(x1,x2) ;  
   
% publish_to_ros(x) ;


%% subscribe a matrix
y = subscribe_from_ros ;
[t,y1] = vec2mat(y) ;

figure
surf(y1/1000,'FaceAlpha',0.5);
view(0,270)
xlim([0 20])
ylim([0 480])
% axis equal 

H = size(y1,1) ;
W = size(y1,2) ;
y2 = zeros(H,1) ;
for h = 1:H
    w_sum = 0 ;
    W_ = 0 ;
    for w = 1:W
        
        % discard invalid pixels
        if y1(h,w)
            W_ = W_ + 1 ;
            w_sum = w_sum + y1(h,w) ;    
        end
        
    end
    
    if W_ % some pixels in width are valid
        y2(h) = w_sum / W_ ;
    else % all pixels in width are invalid
        y2(h) = 0 ;
    end
end

y_mean = sum(y1')'/W ;

figure, hold on;
plot(y_mean,'b','linewidth',2)
plot(y2,'r','linewidth',2)



%% subscribe a vector
%{
y_sub = subscribe_from_ros ;
t = y_sub(1) ;

figure 

plot(y1)
%}



%% FUNCTIONS 
function [] = publish_to_ros(x)

    global pub_type msg_pub % for easy use inside of nested functions
   
    msg_pub.Data = x;

    send( pub_type , msg_pub ) ;

end

function y = subscribe_from_ros

    global sub_type msg_sub % for easy use inside of nested functions

    msg_sub = receive( sub_type , 10 ) ;
    
    y = msg_sub.Data ; 

end


function x = mat2vec(x1,x2) ; 

    nr1 = size(x1,1) ;
    nc1 = size(x1,2) ;
    nr2 = size(x2,1) ;
    nc2 = size(x2,2) ;

    x = [ x1(:) ; x2(:) ]' ;
    x = [ nr1 , nc1 , nr2 , nc2 , x ] ;

end

function [t,y1] = vec2mat(y) 
    
    t = y(1) ;
    nr1 = y(2) ;
    nc1 = y(3) ;
%     nr2 = y(3) ;
%     nc2 = y(4) ;
    
    y = y(4:end) ;
    y1 = reshape(y(1:nr1*nc1),[nr1,nc1]) ;
%     y2 = reshape(y(nr1*nc1+1:end),[nr2,nc2]) ;
end