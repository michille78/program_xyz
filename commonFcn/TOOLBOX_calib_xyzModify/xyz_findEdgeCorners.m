%% fine the four edge corners

function BWCrossIn = xyz_findEdgeCorners( frame,winty,wintx,block_num,isDraw )
%% parameter need to set
if ~exist('block_num','var')
    block_num = 15 ;
end
%%
global whiteThreshold W H 
if ~exist('frame','var')
    frame = imread( '16B_bmp_gray/frame11.bmp' ) ;
%     frame = rgb2gray(frame);
%     frame = imadjust(frame);
    isDraw=1;
end
if ~exist('wintx','var')||~exist('winty','var'),
    wintx = max(round(size(frame,2)/128),round(size(frame,1)/96));
    winty = wintx;
end;
    
isDraw2=1*isDraw;
isText2=1*isDraw;
if isDraw
    figure;
    set(gcf,'outerposition',get(0,'screensize'));
    image(frame);
    colormap(gray(256))
    hold on 
end
edgeCorners = zeros(4,2);   % clockwise

[W,H] = size(frame);    % frame(x,y)    x:[1,W]  y:[1:H]
whiteThreshold = 100 ;


edgeCorners(1,:) = findMinXCorner(frame);
% plot(edgeCorners(1,2),edgeCorners(1,1),'or','markersize',10)

edgeCorners(2,:) = findMaxXCorner(frame);
% plot(edgeCorners(2,2),edgeCorners(2,1),'or','markersize',10)

edgeCorners(3,:) = findMinYCorner(frame);
% plot(edgeCorners(3,2),edgeCorners(3,1),'or','markersize',10)

edgeCorners(4,:) = findMaxYCorner(frame);
% plot(edgeCorners(4,2),edgeCorners(4,1),'or','markersize',10)

% find the precise point
edgeCornersNew = findPreciseDiag( edgeCorners,frame );

% find Black and White cross input

% line( [edgeCornersNew(1,2),edgeCornersNew(3,2)],[edgeCornersNew(1,1),edgeCornersNew(3,1)] )
% line( [edgeCornersNew(2,2),edgeCornersNew(4,2)],[edgeCornersNew(2,1),edgeCornersNew(4,1)] )

% BWCrossIn = findBWCrossIn_ByH( edgeCornersNew,frame ) ;

BWCrossIn = BWCrossInBy_cornerfinder(edgeCornersNew,frame,winty,wintx,block_num );
for i=1:4
    if isDraw2
        plot( BWCrossIn(i,2),BWCrossIn(i,1),'or' ) ;
    end
    if isText2
        text( BWCrossIn(i,2),BWCrossIn(i,1),num2str(i),'fontsize',16,'color','r' ) ;
    end
end
%
figure(gcf)
disp('ok')

function BWCrossIn_k = BWCrossInBy_cornerfinder(edgeCornersNew,frame,winty,wintx,block_num )
m=block_num;    % white or black block number

BWCrossIn_k = zeros(4,2);
if ~exist('winty','var')
    wintx=11;
    winty=11;
end
%% search diag 1
BWCross_diag1 = zeros(28,2);
BWCross_diag1_k = 0;
searchStart = edgeCornersNew(1,:)+( edgeCornersNew(3,:)-edgeCornersNew(1,:) )/(2*m)/2 ;
searchStop = edgeCornersNew(3,:)+( edgeCornersNew(1,:)-edgeCornersNew(3,:) )/(2*m)/2 ;
step = ( searchStop-searchStart ) / (2*m)/2 ;

BWCrossIn_search = searchStart;
search_n=0;
while sign(BWCrossIn_search(1)-searchStart(1))*sign(BWCrossIn_search(1)-searchStop(1)) ~= 1 && search_n<300
    [xc,good,bad,type] = cornerfinder( [BWCrossIn_search(1,2);BWCrossIn_search(1,1)],frame,wintx,winty);
    if good>0 
        if BWCross_diag1_k==0 || ( BWCross_diag1_k>0 && abs( xc(1)-BWCross_diag1(BWCross_diag1_k,2))>1 ) 
            if sign(xc(2)-searchStart(1)) * sign(xc(2)-searchStop(1)) ==-1
               BWCross_diag1_k = BWCross_diag1_k+1 ;
               BWCross_diag1(BWCross_diag1_k,2) = xc(1) ;
               BWCross_diag1(BWCross_diag1_k,1) = xc(2) ;

%                hold on 
%                plot( xc(1),xc(2),'+r' )
            end
        end
    end
    BWCrossIn_search  = BWCrossIn_search + step ;
    search_n=search_n+1;
end
BWCross_diag1 = BWCross_diag1( 1:BWCross_diag1_k,: ) ;

if size(BWCross_diag1,1)~=(2*m-1)
    BWCrossIn_k = [];
    return
else
    BWCrossIn_k(1,:) = BWCross_diag1(1,:) ;
    BWCrossIn_k(3,:) = BWCross_diag1(2*m-1,:) ;
end
%% search diag 2
BWCross_diag2 = zeros(28,2);
BWCross_diag2_k = 0;
searchStart = edgeCornersNew(2,:)+( edgeCornersNew(4,:)-edgeCornersNew(2,:) )/(2*m)/2 ;
searchStop = edgeCornersNew(4,:)+( edgeCornersNew(2,:)-edgeCornersNew(4,:) )/(2*m)/2 ;
step = ( searchStop-searchStart ) / (2*m)/2 ;

BWCrossIn_search = searchStart;
search_n=0;
while sign(BWCrossIn_search(1)-searchStart(1))*sign(BWCrossIn_search(1)-searchStop(1)) ~= 1 && search_n<300
    [xc,good,bad,type] = cornerfinder( [BWCrossIn_search(1,2);BWCrossIn_search(1,1)],frame,wintx,winty);
    if good>0 
        if BWCross_diag2_k==0 || ( BWCross_diag2_k>0 && abs( xc(1)-BWCross_diag2(BWCross_diag2_k,2))>1 ) 
            if sign(xc(2)-searchStart(1)) * sign(xc(2)-searchStop(1)) ==-1
               BWCross_diag2_k = BWCross_diag2_k+1 ;
               BWCross_diag2(BWCross_diag2_k,2) = xc(1) ;
               BWCross_diag2(BWCross_diag2_k,1) = xc(2) ;

%                hold on 
%                plot( xc(1),xc(2),'+r' )
            end
        end
    end
    BWCrossIn_search  = BWCrossIn_search + step ;
    search_n = search_n+1;
end
BWCross_diag2 = BWCross_diag2( 1:BWCross_diag2_k,: ) ;

if size(BWCross_diag2,1)~=(2*m-1)
    BWCrossIn_k = [];
    return
else
    BWCrossIn_k(2,:) = BWCross_diag2(1,:) ;
    BWCrossIn_k(4,:) = BWCross_diag2(2*m-1,:) ;
end

function [ k,b ] = get_k_b( diag1,diag2 )
k = ( diag2(2)-diag1(2) )/( diag2(1)-diag1(1) ) ;
b = diag1(2)-k*diag1(1) ;

%% find the cross point of black and white that input to the calib program
function BWCrossIn_k = findBWCrossIn_ByH( edgeCornersNew,frame )
% frame(x,y) 
m=15;
n=50;
%% four white corners cordinates
whiteCorners0 = [ 0 0; 0 2*m ; 2*m 2*m; 2*m 0 ]'/(2*m) ;   % clockwise
% whiteCorners0 = [ 0 0;  2*m 0 ; 2*m 2*m; 0 2*m ]'; 
whiteCorners0 = [ whiteCorners0 ; ones(1,4) ] ;
%% four black and white cross point that input to the calib program
BWCrossIn0 = [ 1 1; 1 2*m-1; 2*m-1 2*m-1; 2*m-1 1 ]'/(2*m) ;
BWCrossIn0 = [ BWCrossIn0 ; ones(1,4) ] ;

whiteCornersk = [ edgeCornersNew ones(4,1) ]';
HH = whiteCornersk / whiteCorners0  ;

H = compute_homography( whiteCornersk,whiteCorners0 ) ;

BWCrossIn_k = H*BWCrossIn0 ;
BWCrossIn_k = BWCrossIn_k' ;
BWCrossIn_k = BWCrossIn_k(:,1:2) ;

% %  optimization by close to diagonal
% BWCrossIn_k(1,1:2) = optimizationByDiag( edgeCornersNew(1,1:2),edgeCornersNew(3,:),BWCrossIn_k(1,1:2) );
% BWCrossIn_k(3,1:2) = optimizationByDiag( edgeCornersNew(1,1:2),edgeCornersNew(3,:),BWCrossIn_k(3,1:2) );
% BWCrossIn_k(2,1:2) = optimizationByDiag( edgeCornersNew(2,1:2),edgeCornersNew(4,:),BWCrossIn_k(2,1:2) );
% BWCrossIn_k(4,1:2) = optimizationByDiag( edgeCornersNew(2,1:2),edgeCornersNew(4,:),BWCrossIn_k(4,1:2) );

BWCrossIn_k = cornerfinder( BWCrossIn_k',frame',11,11 )' ;



%%  optimization by close to diagonal
function newP = optimizationByDiag( diag1,diag2,P )
k = ( diag2(2)-diag1(2) )/( diag2(1)-diag1(1) ) ;
b = diag1(2)-k*diag1(1) ;
kk = -1/k ;
bb = P(2)-kk*P(1) ;
newP = zeros(1,2);
newP(1) = (bb-b)/(k-kk) ;
cornersCompensate = 0;
if( normest(diag1-P)<normest(diag2-P) )
    % make newP more close to diag1
    newP(1) = newP(1)-sign( newP(1)-diag1(1) )*cornersCompensate;
else
    % make newP more close to diag2
    newP(1) = newP(1)-sign( newP(1)-diag2(1) )*cornersCompensate;
end
newP(2) = k*newP(1)+b ;

function edgeCornersNew = findPreciseDiag( edgeCorners,frame )
global  whiteThreshold W H 
isDraw1=1;
isText1=1;

edgeCornersNew = zeros(4,2);
% find the two farest corners
distance = zeros(1,6);
order = [ 1 2; 1 3; 1 4; 2 3; 2 4; 3 4 ];
for k=1:6
    distance(k) = normest( edgeCorners(order(k,1),:)-edgeCorners(order(k,2),:) ) ;
end
[C, I] = max(distance) ;
% the search line: y = k1_sline*x + b1_sline
% get the slope: k
temp = edgeCorners(order(I,1),:)-edgeCorners(order(I,2),:) ;
k1_sline_temp = temp(2)/temp(1) ;
k1_sline = -1/k1_sline_temp ;
%% search 1 : y = k1_sline*x + b1_sline, from b1_sline_min
% the scope of b1_sline
square = [ 0 0; 0 H; W 0; W H ] ;
b1_sline_square = square(:,2)-k1_sline*square(:,1) ;
b1_sline_min = min(b1_sline_square);
b1_sline_max = max(b1_sline_square);

b1_sline_search = b1_sline_min ;
while b1_sline_search<b1_sline_max
   isWhite = 0 ;
   white_xy_s = zeros(1,2);
   [x_search_scope,y_search_scope] = find_search_scope( k1_sline,b1_sline_search ) ;
%    plot( y_search_scope,x_search_scope,'r' );
    for x = min(x_search_scope):max(x_search_scope)
        y = round(k1_sline*x+b1_sline_search) ;
        if frame(x,y) > whiteThreshold
            isWhite = isWhite+1 ;
            white_xy_s(isWhite,:) = [ x y ];
        end
    end
    if isWhite>0
       break; 
    end
    b1_sline_search = b1_sline_search+1 ;
end
if isWhite==0 
   errordlg(' can not find the point from b1_sline_min'); 
end
edgeCornersNew(1,:) = mean( white_xy_s,1 ) ;

%% search 2: from b1_sline_max
b1_sline_search = b1_sline_max ;
while b1_sline_search>b1_sline_min
   isWhite = 0 ;
   white_xy_s = zeros(1,2);
   [x_search_scope,y_search_scope] = find_search_scope( k1_sline,b1_sline_search ) ;
%    plot( y_search_scope,x_search_scope,'r' );
    for x = min(x_search_scope):max(x_search_scope)
        y = round(k1_sline*x+b1_sline_search) ;
        if frame(x,y) > whiteThreshold
            isWhite = isWhite+1 ;
            white_xy_s(isWhite,:) = [ x y ];
        end
    end
    if isWhite>0
       break; 
    end
    b1_sline_search = b1_sline_search-1 ;
end
if isWhite==0 
   errordlg(' can not find the point from b1_sline_min'); 
end
edgeCornersNew(3,:) = mean( white_xy_s,1 ) ;

%% search 3 : y = k2_sline*x + b2_sline, from b2_sline_min
% the scope of b2_sline
k2_sline = k1_sline_temp ;

b2_sline_square = square(:,2)-k2_sline*square(:,1) ;
b2_sline_min = min(b2_sline_square) ;
b2_sline_max = max(b2_sline_square) ;

b2_sline_search = b2_sline_min ;
while b2_sline_search<b2_sline_max
   isWhite = 0 ;
   white_xy_s = zeros(1,2);
   [x_search_scope,y_search_scope] = find_search_scope( k2_sline,b2_sline_search ) ;
%    plot( y_search_scope,x_search_scope,'r' );
   text( y_search_scope,x_search_scope,'2' );
    for x = min(x_search_scope):max(x_search_scope)
        y = round(k2_sline*x+b2_sline_search) ;
        if frame(x,y) > whiteThreshold
            isWhite = isWhite+1 ;
            white_xy_s(isWhite,:) = [ x y ];
        end
    end
    if isWhite>0
       break; 
    end
    b2_sline_search = b2_sline_search+1 ;
end
if isWhite==0 
   errordlg(' can not find the point from b2_sline_min'); 
end
edgeCornersNew(2,:) = mean( white_xy_s,1 ) ;

%% search 4 : y = k2_sline*x + b2_sline, from b2_sline_max
% the scope of b2_sline

b2_sline_square = square(:,2)-k2_sline*square(:,1) ;
b2_sline_min = min(b2_sline_square) ;
b2_sline_max = max(b2_sline_square) ;

b2_sline_search = b2_sline_max ;
while b2_sline_search>b2_sline_min
   isWhite = 0 ;
   white_xy_s = zeros(1,2);
   [x_search_scope,y_search_scope] = find_search_scope( k2_sline,b2_sline_search ) ;
%    plot( y_search_scope,x_search_scope,'r' );
    text( y_search_scope,x_search_scope,'4' );
    for x = min(x_search_scope):max(x_search_scope)
        y = round(k2_sline*x+b2_sline_search) ;
        if frame(x,y) > whiteThreshold
            isWhite = isWhite+1 ;
            white_xy_s(isWhite,:) = [ x y ];
        end
    end
    if isWhite>0
       break; 
    end
    b2_sline_search = b2_sline_search-1 ;
end
if isWhite==0 
   errordlg(' can not find the point from b2_sline_min'); 
end
edgeCornersNew(4,:) = mean( white_xy_s,1 ) ;
for i=1:4
    if isDraw1
    plot(edgeCornersNew(i,2),edgeCornersNew(i,1),'ob')
    end
    if isText1
    text(edgeCornersNew(i,2),edgeCornersNew(i,1),num2str(i),'fontsize',17,'color','r')
    end
end

%% fine the cross point of y=k*x+b and the four square edge line
% then get the search point of x
function [x_search_scope,y_search_scope] = find_search_scope(k,b)
global W H
if abs(k)<1e-3
    x_search_scope = [1 W];
    return;
end
crossPoint = [ (2-b)/k 2; 2 b+2*k; W-1 k*(W-1)+b; (H-1-b)/k H-1 ] ;
crossPoint = round(crossPoint);
k = 1 ;
while k<=size(crossPoint,1)
   if crossPoint(k,1)<1 || crossPoint(k,1)>W || crossPoint(k,2)<1 || crossPoint(k,2)>H
      crossPoint(k,:)  = []; 
   else
       k = k+1 ;
   end   
end
x_search_scope = crossPoint(:,1);
y_search_scope = crossPoint(:,2);

function edgeCorners_Xmin = findMinXCorner( frame )
% the edgeCorner that x is minimum 
%   ( if there are more than one, chose the minmum y  when y is smaller than H/2
%        chose the maximum y when y is bigger than H/2   )

global  whiteThreshold W H 

x = 1 ;
y_s = zeros(1,10) ;
while x<W 
    isWhite=0;
    for y=1:H
       if frame(x,y) > whiteThreshold % confirm white
           isWhite = isWhite+1 ;
           y_s(isWhite) = y ;
       end
    end
    y_s = y_s(1:isWhite);
    if isWhite>0
       break; 
    end
    x = x+1;
end
if x>=W || isWhite==0
   errordlg('con not find the edgeCorner that x is minimum ') ; 
   return;
end
if mean(y_s)>H/2
    y_corner = max(y_s);
else
    y_corner = min(y_s);
end
edgeCorners_Xmin = [ x,y_corner ];

function edgeCorners_Xmax = findMaxXCorner( frame )
% the edgeCorner that x is maximum 
%   ( if there are more than one, chose the minmum y  when y is smaller than H/2
%        chose the maximum y when y is bigger than H/2   )
global  whiteThreshold W H 

x = W ;
y_s = zeros(1,10) ;
while x>1 
    isWhite=0;
    for y=1:H
       if frame(x,y) > whiteThreshold % confirm white
           isWhite = isWhite+1 ;
           y_s(isWhite) = y ;
       end
    end
    y_s = y_s(1:isWhite);
    if isWhite>0
       break; 
    end
    x = x-1;
end
if x<3 || isWhite==0
   errordlg('con not find the edgeCorner that x is maximum ') ; 
   return;
end
if mean(y_s)>H/2
    y_corner = max(y_s);
else
    y_corner = min(y_s);
end
edgeCorners_Xmax = [x y_corner];

function edgeCorners_Ymin = findMinYCorner( frame )
% the edgeCorner that y is minimum 
%   ( if there are more than one, chose the minmum x  when x is smaller than W/2
%        chose the maximum x when x is bigger than W/2   )
global  whiteThreshold W H 

y = 1 ;
x_s = zeros(1,10) ;
while y<H 
    isWhite=0;
    for x=1:W
       if frame(x,y) > whiteThreshold % confirm white
           isWhite = isWhite+1 ;
           x_s(isWhite) = x ;
       end
    end
    x_s = x_s(1:isWhite);
    if isWhite>0
       break; 
    end
    y = y+1;
end
if y>=H || isWhite==0
   errordlg('con not find the edgeCorner that y is minimum ') ; 
   return;
end
if mean(x_s)>H/2
    x_corner = max(x_s);
else
    x_corner = min(x_s);
end

edgeCorners_Ymin = [x_corner y ];

function edgeCorners_Ymax = findMaxYCorner( frame )
% the edgeCorner that y is maximum 
%   ( if there are more than one, chose the minmum x  when x is smaller than W/2
%        chose the maximum x when x is bigger than W/2   )
global  whiteThreshold W H 

y = H ;
x_s = zeros(1,10) ;
while y>1
    isWhite=0;
    for x=1:W
       if frame(x,y) > whiteThreshold % confirm white
           isWhite = isWhite+1 ;
           x_s(isWhite) = x ;
       end
    end
    x_s = x_s(1:isWhite);
    if isWhite>0
       break; 
    end
    y = y-1;
end
if y<3 || isWhite==0
   errordlg('con not find the edgeCorner that y is minimum ') ; 
   return;
end
if mean(x_s)>H/2
    x_corner = max(x_s);
else
    x_corner = min(x_s);
end
edgeCorners_Ymax = [x_corner  y ];
