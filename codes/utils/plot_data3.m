function plot_data2(data, Rw_b, Tw_b, points, XYZw)

pts_w = [];
pts_c = [];
for j = 1:numel(data.id)
    id_j = data.id(j);
    [p0, p1, p2, p3, p4] = get_tag_coords(id_j);
    % coordinates in the world frame
    pts = [p0, p1, p2, p3, p4];
    pts = [pts; zeros(1,5)];
    pts_w = [pts_w, pts];

    % coordinates in picture
    pts = [data.p0(:,j), data.p1(:,j), data.p2(:,j), data.p3(:,j), data.p4(:,j)];
    pts_c = [pts_c, pts];
end

subplot(1,2,1); 
imshow(data.img); 
hold on;
scatter(points(:,1), points(:,2), '*');
hold off;
subplot(1,2,2); 
Rxpi = [1, 0, 0; 0, cos(pi), -sin(pi); 0, sin(pi), cos(pi)];
plotCamera('Location', Tw_b, 'Orientation', Rw_b*Rxpi, 'Opacity',0, 'Size', 0.1);
hold on;
scatter3(pts_w(1,:), pts_w(2,:), pts_w(3,:));
scatter3(XYZw(1,:), XYZw(2,:), XYZw(3,:));
rectangle('Position',[0 0 3.4960 2.6360]);
xlim([-1, 3.5]);
ylim([-1, 3]);
zlim([0, 3]);
xlabel('x');
ylabel('y');
zlabel('z');
hold off; 
drawnow;