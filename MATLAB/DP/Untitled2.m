s = 5;
V = 25;
t = zeros(s, 2*s);
for i=1:s
    r = ((i-1)*s + 1):i*s;
    t(i,i:(i+s-1)) = r; 
end

rows_to_do = [];
for i=1:2*s
   l = t(find(t(:,i)),i);
   rows_to_do(end+1:end+(length(l))) = l;
end
rows_to_do = fliplr(rows_to_do);