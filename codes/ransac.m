function [v_omega_best] = ransac(A_all, x, y, Zc, dot_x, dot_y)
    ssd = @(x, y) (x-y).^2;
    N = numel(x);
    max_inliers = zeros(N, 1);
    thresh = 1e-3;
    for t = 1:20
        r_idx = randi([1,N], 6, 1);
        A = get_A(x(r_idx)',y(r_idx)',Zc(r_idx)');
        v_omega_c = A\[dot_x(r_idx);dot_y(r_idx)];
        ssd_vec = ssd([dot_x;dot_y], A_all*v_omega_c);
        ssd_vec = ssd_vec(1:N) + ssd_vec(N+1:end);
        inliers = ssd_vec < thresh;
        if sum(inliers) > sum(max_inliers)
            max_inliers = inliers;
        end
    end
%     sum(max_inliers)
    A = get_A(x(max_inliers)',y(max_inliers)',Zc(max_inliers)');
    v_omega_best = A\[dot_x(max_inliers);dot_y(max_inliers)];
end