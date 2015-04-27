%% xyz 2015 3.18

function ellipseTest(  )

cx = 10;
cy = 12;
a = 60 ;
b = 50 ;
thita = -30*pi/180 ;  % rad
f = sqrt( abs(a^2-b^2) ) ;
e = f/max(a,b) ;

[ A, B, C, D, E, F ] = ellipseStandToPoly(  cx,cy,thita,a,b  ) ;
ePoly = [ A, B, C, D, E, F ] ;
% % dbstop in ellipsePolyToStand
[ cx_f,cy_f,thita_f,a_f,b_f,e_f,f_f ] = ellipsePolyToStand( ePoly ) ;
figure
axesm mercator
[elat,elon] = ellipse1( cx,cy,[ a,e ],thita);
plotm(elat,elon,'k')

hold on
[elat,elon] = ellipse1( cx_f(1),cy_f(1),[ a_f(1),e_f(1) ],thita_f(1));
plotm(elat,elon,'r')

hold on
[elat,elon] = ellipse1( cx_f(2),cy_f(2),[ a_f(2),e_f(2) ],thita_f(2));
plotm(elat,elon,'g')

%% test EllipseFitfing
%%% generate ellipse data 
H = GenerateEllipseData( ePoly ) ;
%%% add noise
N = size(H,1) ;
temp = max(H)-min(H) ;
mu = 0;
sigma = sqrt(temp/50)*0 ;
HxNoise = normrnd(mu,sigma(1),[N 1]);
HyNoise = normrnd(mu,sigma(2),[N 1]);
H = H+[HxNoise HyNoise];
%% Ellipse Fitfing
[ ePoly_Fitting,eStand ] = EllipseFitfing( H ) ;
for i=1:size(ePoly_Fitting,2)
    [HerrMean(i),Herr(:,i)] = CheckData( ePoly_Fitting(:,i),H ) ;
end
for i=1:size(ePoly_Fitting,2)
    [ A_f(1,i), B_f(1,i), C_f(1,i), D_f(1,i), E_f(1,i), F_f(1,i) ] = deal( ePoly_Fitting(1,i),ePoly_Fitting(2,i),ePoly_Fitting(3,i),ePoly_Fitting(4,i),ePoly_Fitting(5,i),ePoly_Fitting(6,i) ) ;
    H_Fiting{i} = GenerateEllipseData( ePoly_Fitting ) ;  % Fitfing Ellipse point
end
for i=1:size(eStand,2)
    [ cx_f(1,i),cy_f(1,i),thita_f(1,i),a_f(1,i),b_f(1,i),e_f(1,i),f_f(1,i) ] = deal( eStand(1,i),eStand(2,i),eStand(3,i),eStand(4,i),eStand(5,i),eStand(6,i),eStand(7,i) ) ;
end

%% draw
figure('name','Poly_Fitting_Points')
% axesm mercator
plot(H(:,1),H(:,2),'*g')
hold on
strFitting = '';
mark = { '.r','ob' };
for i=1:size(ePoly_Fitting,2)
    plot(H_Fiting{i}(:,1),H_Fiting{i}(:,2),mark{i});
    strFitting{i}=sprintf( 'fitting-%d',i );
end
% [elat,elon] = ellipse1( cx,cy,[ max(a,b),e ],thita );
% plot(elat,elon,'k')

legend([{'original points'},strFitting])

figure
axesm mercator
[elat,elon] = ellipse1( cx,cy,[ a,e ],thita);
plotm(elat,elon,'.k')
hold on

strFitting = '';
mark = { 'r','b','g','y' };
for i=1:size(eStand,2)
    [elat,elon] = ellipse1( cx_f(1,i),cy_f(1,i),[ a_f(1,i),e_f(1,i) ],thita_f(1,i));
    plotm(elat,elon,mark{i})    
    strFitting{i}=sprintf( 'fittingS-%d',i );
end
legend([{'original'},strFitting])

disp('test OK')

function [HerrMean,Herr] = CheckData( ePoly,H )
N = size(H,1);
Herr = zeros(N,1);

for k=1:N
    x = H(k,1) ;
    y = H(k,2) ;
    [ A, B, C, D, E, F ] = deal( ePoly(1),ePoly(2),ePoly(3),ePoly(4),ePoly(5),ePoly(6) );
    Herr(k)  = A*x^2 + B*x*y + C*y^2 + D*x + E*y + F ;
end
HerrMean = mean(abs(Herr),1) ;

function multiply = calMultiple( A,B,C,F,cx,cy,thita )
t = A*cx^2+B*cx*cy+C*cy^2-F ;
multiply = 4*t/( (A+C)^2-B^2/sin(thita)^2 );

