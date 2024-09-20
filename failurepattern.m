warning('off');
X1=X;
Elem_X= ElemX;
Elem_Y= ElemY;

% X=X1;
% ElemX=Elem_X;
% ElemY=Elem_Y;

B = 1;
ElemX_temp = [];
ElemY_temp = [];

row = 1;
for i = 1:3:size(ElemX)
    ElemX_temp(row, :) = ElemX(i:i+2, 1)';
    ElemY_temp(row, :) = ElemY(i:i+2, 1)';
    row = row + 1;
end
ElemX = ElemX_temp;
ElemY = ElemY_temp;
EX = ElemX';
EY = ElemY';

phi = fi * (pi / 180);
c = c;
for i=1:3*size(ElemX,1)
    sigma1(i)=X(3*i-2);
    sigma2(i)=X(3*i-1);
    sigma3(i)=X(3*i);
    ac(i)=(sigma1(i)-sigma2(i))^2+(2*sigma3(i))^2;
    dc(i)=(2*c*cos(phi)-(sigma1(i)+sigma2(i))*sin(phi))^2;
    kk(i)=ac(i)/dc(i);
end
for i=1:3*size(ElemX,1)
    af(i)=EX(i);
    bf(i)=EY(i);
end
% ag(:,1)=af;
% ag(:,2)=bf;
% ag(:,3)=kk;
ag(:,1:2) = final_nodes_array(:,1:2);
ag(:,3) = kk;
x=ag(:,1);
Y=ag(:,2);
z=ag(:,3);

x=(x)/B;
Y=(Y)/B;

x=round(x*1e8)/1e8;
Y=round(Y*1e8)/1e8;
xmin=min(x);
ymin=min(Y);
xmax=max(x);
ymax=max(Y);
xres=100;
yres=100;
figure;


hold on;
xv=linspace(xmin,xmax,xres);
yv=linspace(ymin,ymax,yres);
[Xinterp,Yinterp]=meshgrid(xv,yv);
Zinterp=griddata(x,Y,z,Xinterp,Yinterp);
surface(Xinterp,Yinterp,Zinterp);


grid off
axis equal
shading faceted
shading interp
nc=64;
temp=0.96;
for i=1:nc
    ncol(i,1:3)=temp;
    temp=temp-1/nc;
    if temp<=0
        temp=0;
    end
end

axis equal
colormap_array = red_to_cyan(264);
colormap(flipud(colormap_array));
% colormap("jet");

hcol=colorbar;
cpos=get(hcol,'Position');
cpos(4)=cpos(4)/2;
cpos(3)=cpos(3)/2;
cpos(1)=cpos(1)+0.105;
cpos(2)=cpos(2)+0.075;
set(hcol,'Position',cpos)
late=max(x);
clear x y;
hold on
Zval = max(Zinterp(:)) + 1;

footing_x = [L,L,L+B,L+B,L];
footing_y = [0,-Df,-Df,0,0];
fill_footing_region_3D(footing_x,footing_y,Zval);
fill_circle_region_3D(tunnel_interface, Zval);

xlabel('x/B','Fontweight','bold','Fontsize',15);
ylabel('y/B','Fontweight','bold','Fontsize',15);
set(gca,'Fontsize',16);
axis([xmin-0.15 xmax ymin-0.15 ymax])
% set(gcf,'PaperType', 'A4')
% set(gca,'XTick',[-12 -10 -8 -6 -4 -2 0 2 4 6 8 10 12]);
% % set(gca,'XTickLabel','-12|-10|-8|-6|-4|-2|0|2|4|6|8|10|12')
% set(gca,'YTick',[-8 -6 -4 -2 0]);
% set(gca,'YTickLabel','-8|-6|-4|-2|0')
box off;

figuresFolder = fullfile(subFolderName, 'figures');
if ~exist(figuresFolder, 'dir')
    mkdir(figuresFolder);  % Create the folder if it doesn't exist
end
fileName_figure = fullfile(figuresFolder, ['b_B_' num2str(b_by_B) '_d_B_' num2str(d_by_B) '_fi_' num2str(phiii) '.fig']);
savefig(fileName_figure);  

function fill_footing_region_3D(x,y,Zval)
    hold on;
    patch(x, y, Zval * ones(size(x)), 'w', 'FaceAlpha', 1, 'EdgeColor', 'b');
    
    axis equal;
    grid on;
 
end
function fill_circle_region_3D(data,Zval)
    hold on;
    x = data(:,1);
    y = data(:,2);
    x(end+1) = x(1);
    y(end+1) = y(1);
    patch(x, y, Zval * ones(size(x)), 'w', 'FaceAlpha', 1, 'EdgeColor', 'b');   
    axis equal;
    grid on;
 
end
function cmap = red_to_cyan(n)
    % Check if the input argument n is provided
    if nargin < 1
        n = 64; % Default number of colors
    end
    
    
    cmap = zeros(n, 3);
    
    for i = 1:n
        t = (i-1) / (n-1); % normalized position in the colormap
        cmap(i, 1) = 1 - t;   % Red channel decreases from 1 to 0
        cmap(i, 2) = t;       % Green channel increases from 0 to 1
        cmap(i, 3) = t;       % Blue channel increases from 0 to 1
    end
end
