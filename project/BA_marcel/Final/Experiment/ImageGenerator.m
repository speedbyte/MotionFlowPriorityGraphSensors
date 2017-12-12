
if  1 == 1

mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_slow');
mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_slow/no_noise');
mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_slow/static_FG');
mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_slow/static_BG');
mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_slow/dynamic_FG');
mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_slow/dynamic_BG');

speed = 0.5;
slow = init(speed,1);
generate(slow,0,'slow/no_noise');
generate(slow,1,'slow/dynamic_BG');
generate(slow,2,'slow/static_BG');
generate(slow,3,'slow/static_FG');
generate(slow,4,'slow/dynamic_FG');
end

%
%  

if  1 == 0
    
mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_normal');
mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_normal/no_noise');
mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_normal/static_FG');
mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_normal/static_BG');
mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_normal/dynamic_FG');
mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_normal/dynamic_BG');

speed = 2;
normal = init(speed,2);
generate(normal,0,'normal/no_noise');
generate(normal,1,'normal/dynamic_BG');
generate(normal,2,'normal/static_BG');
generate(normal,3,'normal/static_FG');
generate(normal,4,'normal/dynamic_FG');

end

if  1 == 0
mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_fast');
mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_fast/no_noise');
mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_fast/static_FG');
mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_fast/static_BG');
mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_fast/dynamic_FG');
mkdir('./../../../../matlab_dataset/data/stereo_flow/image_02_fast/dynamic_BG');

speed = 6;
fast = init(speed,3);
generate(fast,0,'fast/no_noise');
generate(fast,1,'fast/dynamic_BG');
generate(fast,2,'fast/static_BG');
generate(fast,3,'fast/static_FG');
generate(fast,4,'fast/dynamic_FG');

end