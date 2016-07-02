v = VideoWriter('pipeline','MPEG-4' );
v.FrameRate = 75;
v.Quality = 50;
open(v)

for ii = 1:length(M)
    writeVideo(v, M(ii));
end

close(v)