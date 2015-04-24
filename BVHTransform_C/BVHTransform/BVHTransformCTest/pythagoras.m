function c = pythagoras(a,b,color) %#codegen
% Calculates the hypotenuse of a right triangle
%  and displays the triangle as a patch object. 

c = sqrt(a^2 + b^2);

create_plot(a, b, color);

function create_plot(a, b, color)
%Declare patch and axis as extrinsic

% coder.extrinsic('patch', 'axis'); 

x = [0;a;a];
y = [0;0;b];
patch(x, y, color);
axis('equal');



