function P = get_tag_coords2(ids)
% @input    id                  1xn vector indicating the indices
% @output   P                   2x(5n) xy coordinates of the tag[id]

% P0-4s               precomputed coordinates
persistent P0s;
persistent P1s;
persistent P2s;
persistent P3s;
persistent P4s;

if isempty(P0s)
P0s = zeros(2, 108);
P1s = zeros(2, 108);
P2s = zeros(2, 108);
P3s = zeros(2, 108);
P4s = zeros(2, 108);
for i = 0:107
    [p0, p1, p2, p3, p4] = get_tag_coords(i);
    P0s(:,i+1) = p0;
    P1s(:,i+1) = p1;
    P2s(:,i+1) = p2;
    P3s(:,i+1) = p3;
    P4s(:,i+1) = p4;
end 
end

ids = ids+1;
P = [P0s(:,ids), P1s(:,ids), P2s(:,ids), P3s(:,ids), P4s(:,ids)];
