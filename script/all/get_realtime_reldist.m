function [dt, dist] = get_realtime_reldist(x1, y1, t1, x2, y2, t2)

size_x1 = max(size(x1));
size_x2 = max(size(x2));
if size_x1 <= size_x2
    for i = 1 : max(size(x1))
        t1_curr = t1(i);
        [min_dt, index_dt] = min(abs(t2(:,1) - t1_curr));
        dx = x1(i) - x2(index_dt);
        dy = y1(i) - y2(index_dt);
        dist(i,:) = sqrt(dx*dx + dy*dy);
    end
    dt = t1(:,1) - t1(1);
else
    for i = 1 : max(size(x2))
        t2_curr = t2(i);
        [min_dt, index_dt] = min(abs(t1(:,1) - t2_curr));
        dx = x2(i) - x1(index_dt);
        dy = y2(i) - y1(index_dt);
        dist(i,:) = sqrt(dx*dx + dy*dy);
    end
    dt = t2(:,1) - t2(1);
end

end