clear 
for iteration = 1:200
    

import casadi.*


A = 4*(rand(2)-0.5);
B = [1 ; 0];


Nt = 200;
T = 10;
tspan = linspace(0,T,Nt);
%
%%

xs = SX.sym('xs',2,1);
us = SX.sym('us',1,1);
ts = SX.sym('t');

Fs = Function('f',{ts,xs,us},{A*xs+B*us});

%%
Nt = length(tspan);
%%
us_t = SX.sym('ft',[1 Nt]);
%
x0 = 4*(rand(2,1)-0.5);

xs_t   = x0; 

x_next = xs_t;
for it = 2:Nt
    dt = tspan(it) - tspan(it-1);
    % rk4
    k1 =  Fs(tspan(it-1)        , x_next             , us_t(:,it-1));
    k2 =  Fs(tspan(it-1)+0.5*dt , x_next + 0.5*k1*dt , 0.5*us_t(:,it-1) + 0.5*us_t(:,it));
    k3 =  Fs(tspan(it-1)+0.5*dt , x_next + 0.5*k2*dt , 0.5*us_t(:,it-1) + 0.5*us_t(:,it));
    k4 =  Fs(tspan(it-1)+dt     , x_next + 1.0*k3*dt , us_t(:,it));
    x_next = x_next + (1/6)*dt*(k1 + 2*k2 + 2*k3 + k4);
    %
    xs_t   = [xs_t x_next]; 
end

%%

Fs_intergrated = casadi.Function('Fs_int',{xs,us_t},{xs_t});
%%
xTarget = 4*(rand(2,1)-0.5);
% 
%Psi = (x_next - xTarget)'*(x_next-xTarget);

sm_abs = @(x) sqrt(x.^2+1e-5);
sm_max = @(x,y) 0.5*(x + y + sm_abs(x-y));

epsilon = 1e-3;
PathCost = @(x) sm_max(0.5,sum((x - xTarget).^2));
L = @(x,u)  PathCost(x)+ epsilon*(u.^2);

L_integral   = sum(diff(tspan).*(L(xs_t(:,1:end-1),us_t(1:end-1)) + L(xs_t(:,2:end),us_t(2:end)))/2);


 nlp = struct( 'x' , us_t(:)   , ...
               'f' ,  L_integral   , ...
               'g' , []        );
%           
opt = struct('ipopt',struct('print_level',0,'tol',1e-12));
S = nlpsol('S', 'ipopt', nlp,opt);
% 
%%
r = S('x0',zeros(size(us_t)));
%%
clf
subplot(2,2,1)
hold on
u_opt = full(r.x);
plot(u_opt)
ip_control = plot(u_opt(1),'Marker','.','Color','k','MarkerSize',15);
title('Control')
subplot(2,2,2)
hold on
xt_opt = full(Fs_intergrated(x0,r.x))';
plot(xt_opt)
ip_state_1 = plot(1,xt_opt(1,1),'Marker','.','Color','k','MarkerSize',15);
ip_state_2 = plot(1,xt_opt(1,2),'Marker','.','Color','k','MarkerSize',15);

legend('x_1','x_2')
title('States')

%%
subplot(2,2,[3 4])
x1span = linspace(-10,10,100);
x2span = linspace(-10,10,100);
[xmesh ,ymesh] = meshgrid(x1span,x2span);

points = [xmesh(:) ymesh(:)]';
Zvalues = PathCost(points);
Zvalues = reshape(Zvalues,100,100);
hold on
contour(xmesh,ymesh,Zvalues,'LineWidth',0.05,'LevelStep',4)
title('Final Cost')
caxis([-5 20])
colormap(jet(50))
view(0,-90)
text(0,0,num2str(A(1,1),'%.2e'))
text(3,0,num2str(A(1,2),'%.2e'))
text(0,3,num2str(A(2,1),'%.2e'))
text(3,3,num2str(A(2,2),'%.2e'))

ei = eig(A);
text(6,6,num2str(ei(1),'%.2e'))
text(6,7,num2str(ei(2),'%.2e'))

shading interp
colorbar

plot(xt_opt(:,1),xt_opt(:,2),'LineWidth',2)
plot(xt_opt(1,1),xt_opt(1,2),'Marker','.','Color','red','MarkerSize',25)
plot(xt_opt(end,1),xt_opt(end,2),'Marker','p','Color','b','MarkerSize',25)

ip = plot(xt_opt(1,1),xt_opt(1,2),'Marker','.','Color','k','MarkerSize',35);

xlim([-10 10])
ylim([-10 10])

v = VideoWriter("videos/"+iteration+floor(rand*1000)+".mp4",'MPEG-4');
open(v)

for it = 1:Nt
    ip.XData = xt_opt(it,1);
    ip.YData = xt_opt(it,2);
    %
    ip_control.XData = it;
    ip_state_1.XData = it;
    ip_state_2.XData = it;
    %
    ip_control.YData = u_opt(it);
    ip_state_1.YData = xt_opt(it,1);
    ip_state_2.YData = xt_opt(it,2);
    %
   frame = getframe(gcf);
   writeVideo(v,frame);

end 

close(v);

end

%% https://drive.google.com/drive/folders/1dccXoXf85cscBr5FFRyhv9euvQbNU-4r