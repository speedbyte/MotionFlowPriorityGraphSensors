% compiles all mex files used for object scene flow computation

%% Computation of data term
mex ssf/unary/computeDataTermMex.cpp ./roadplane/matrix.cpp CXXFLAGS="$CXXFLAGS -DMM_POPCNT -msse4.2 -fPIC" -outdir build

%% Computation of pairwise term
mex ssf/pairwise/computeSmoothnessTermMex.cpp ./roadplane/matrix.cpp -outdir build

%% Roadplane estimation
mex ./roadplane_disp/computeRoadPlaneEstimateMex.cpp -outdir build

%% Background interpolation
mex ./util/backgroundInterpolationMex.cpp -outdir build

%% libviso2
mex ./external/libviso2/matlab/matcherMex.cpp ./external/libviso2/src/matcher.cpp ./external/libviso2/src/filter.cpp ./external/libviso2/src/triangle.cpp ./external/libviso2/src/matrix.cpp -I"./external/libviso2/src" CXXFLAGS="$CXXFLAGS -msse3 -fPIC" -outdir build
mex ./external/libviso2/matlab/visualOdometryStereoMex.cpp ./external/libviso2/src/viso_stereo.cpp ./external/libviso2/src/viso.cpp ./external/libviso2/src/matcher.cpp ./external/libviso2/src/filter.cpp ./external/libviso2/src/triangle.cpp ./external/libviso2/src/matrix.cpp -I"./external/libviso2/src" CXXFLAGS="$CXXFLAGS -msse3 -fPIC" -outdir build

%% libtrws
mex ./external/libtrws/trwsMex.cpp -largeArrayDims CXXFLAGS="\$CXXFLAGS -std=c++0x -fpermissive" -outdir build

%% spsstereo
mex ./external/libstereo/spsstereo/spsStereoMex.cpp ...
    ./external/libstereo/spsstereo/SPSStereo.cpp ./external/libstereo/spsstereo/SGMStereo.cpp ...
    -I"./external/libstereo/spsstereo/" ...
    CXXFLAGS="$CXXFLAGS -DMM_POPCNT -msse4.2 -fPIC" -lpng -outdir build

