function [sc,p] = restplot(H,ang,cbticks,Lval,Lrel,leg,mrk)
%% Init
% Set the ticks of the colorbar based on the interval if given
if ~exist('cbticks','var') || isempty(cbticks)
    cbticks = -180:30:180;
end

% Compose a color vector indicating the orientation of each rest pose
Cs = parula(100);
for ii = 1:size(H,3)
    [~, c(ii)] = min(abs(rad2deg(ang(ii))-linspace(cbticks(1),cbticks(end),length(Cs))));
    C(ii,:) = Cs(c(ii),:);
end

% Find the bounds of the position
[S,L] = bounds(squeeze(H(1:2,4,:)),2);
axdiff = max(L-S);

% Set the length of the little line indicating orientation
if exist('Lval','var') && ~isempty(Lval)
    if exist('Lrel','var') && ~isempty(Lrel) && Lrel
        l = axdiff*Lval;       % If relative, relate to the axis width
    else
        l = Lval;              % Otherwise, use absolute value
    end
else
    l = axdiff/50;          % Default lenght if none given
end

% Set the marker to plot the rest position
if ~exist('mrk','var') || isempty(mrk)
    mrk = '.';
end

% Two points in local frame of the box
v1 = [0;0;0;1]; % rest position, origin of box frame
v2 = [l;0;0;1]; % position translated l along local x axis

%% Plotting
hold on;
for ii = 1:size(H,3)
    v = [squeeze(H(:,:,ii))*v1, squeeze(H(:,:,ii))*v2];         % points in world frame
    sc = scatter(v(1,1),v(2,1),mrk,'MarkerEdgeColor',C(ii,:));  % point indication position
    p = plot(v(1,:),v(2,:),'Color',C(ii,:));                    % line indicating orientation
end
% Add a colorbar related to the angles
colorbar('Ticks',cbticks)
caxis(cbticks([1,end]));

% If specificied, add a legend
if exist('leg','var') && ~isempty(leg) && leg
    legend('x-axis direction','Rest position','Location','SouthEast')
end

% Set axis limits
% xlim(mean([S(1),L(1)]) + [-0.6*axdiff, 0.6*axdiff])
% axis equal

end