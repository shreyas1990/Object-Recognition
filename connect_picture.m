function connect_picture(W_X, color)
% Given 4 points draws 4 lines connecting them like a polygon

if(nargin<2)
    color = 'b';
end
if(size(W_X,1) == 2)
    for i=1:4
        line([W_X(1,i) W_X(1,mod(i,4)+1)], [W_X(2,i) W_X(2,mod(i,4)+1)], 'Color', color);
    end
else
    for i=1:4
        line([W_X(1,i) W_X(1,mod(i,4)+1)], [W_X(2,i) W_X(2,mod(i,4)+1)], [W_X(3,i) W_X(3,mod(i,4)+1)], 'Color', color);
    end
end