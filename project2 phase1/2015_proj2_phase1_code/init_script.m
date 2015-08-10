% add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_pose_handle = @(sensor) estimate_pose(sensor, your_input);
% We will only call estimate_pose_handle in the test function.
% Note that unlike project 1 phase 3, thise will only create a function
% handle, but not run the function at all.
% 
% for j=1:size(data,2)
%     imshow(data(1,j).img)
%      M(j) = getframe;
% end
% movie(M)
a = 0.152;
b = 0.178;
 world  = cell(24,18);
for j=1:5
    for i = 1:24
        
        world{i,j} = [ (((i-1)*a + i*a)/2), (((j-1)*a +j*a )/2) ; 
            (i-1)*a,(j-1)*a; 
            i*a, (j-1)*a;
            i*a, j*a ;
            (i-1)*a,j*a];
    end 
 end

for j=6
    for i = 1:24
        
        world{i,j} = [ (((i-1)*a + i*a)/2), (((j-1)*a + ((j-1)*a +b) )/2) ;
            (i-1)*a,(j-1)*a; 
            i*a, (j-1)*a;
            i*a, (j-1)*a + b ;
            (i-1)*a,(j-1)*a + b];
    end 
end


for j=7:11
    for i = 1:24
        
       world{i,j} = [ (((i-1)*a + i*a)/2), (((j-2)*a +b +(j-1)*a +b)/2) ;
           (i-1)*a,(j-2)*a + b; 
            i*a, (j-2)*a + b;
            i*a, (j-1)*a + b ;
            (i-1)*a,(j-1)*a + b];
    end 
end

for j=12
    for i = 1:24
        
        world{i,j} = [ (((i-1)*a + i*a)/2), (((j-2)*a +(j-2)*a +3*b)/2) ;
            (i-1)*a,((j-2)*a +b); 
            i*a, ((j-2)*a +b );
            i*a, ((j-2)*a + 2*b) ;
            (i-1)*a,((j-2)*a + 2*b)];
   end
end
            
  

for j=13:18
    for i = 1:24
        
       world{i,j} = [ (((i-1)*a + i*a)/2), (((j-3)*a +(j-2)*a + 4*b)/2) ;
       (i-1)*a,(j-3)*a + 2*b; 
            i*a, (j-3)*a + 2*b;
            i*a, (j-2)*a + 2*b;
            (i-1)*a,(j-2)*a + 2*b];
    end 
end

 worldnew = world (1:2:24, 1:2: 18);
figure,
 axis([0 4 0 4])
 for i = 1:108
     
plot(worldnew{i}(1,1)',worldnew{i}(1,2)','o')
hold on
drawnow;
% pause(0.25)
 end
estimate_pose_handle = @(sensor) estimate_pose(sensor,worldnew);

