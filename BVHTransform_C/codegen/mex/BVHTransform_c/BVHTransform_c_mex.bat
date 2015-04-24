@echo off
set MATLAB=D:\Program Files\MATLAB\R2014a
set MATLAB_ARCH=win64
set MATLAB_BIN="D:\Program Files\MATLAB\R2014a\bin"
set ENTRYPOINT=mexFunction
set OUTDIR=.\
set LIB_NAME=BVHTransform_c_mex
set MEX_NAME=BVHTransform_c_mex
set MEX_EXT=.mexw64
call mexopts.bat
echo # Make settings for BVHTransform_c > BVHTransform_c_mex.mki
echo COMPILER=%COMPILER%>> BVHTransform_c_mex.mki
echo COMPFLAGS=%COMPFLAGS%>> BVHTransform_c_mex.mki
echo OPTIMFLAGS=%OPTIMFLAGS%>> BVHTransform_c_mex.mki
echo DEBUGFLAGS=%DEBUGFLAGS%>> BVHTransform_c_mex.mki
echo LINKER=%LINKER%>> BVHTransform_c_mex.mki
echo LINKFLAGS=%LINKFLAGS%>> BVHTransform_c_mex.mki
echo LINKOPTIMFLAGS=%LINKOPTIMFLAGS%>> BVHTransform_c_mex.mki
echo LINKDEBUGFLAGS=%LINKDEBUGFLAGS%>> BVHTransform_c_mex.mki
echo MATLAB_ARCH=%MATLAB_ARCH%>> BVHTransform_c_mex.mki
echo BORLAND=%BORLAND%>> BVHTransform_c_mex.mki
echo OMPFLAGS= >> BVHTransform_c_mex.mki
echo OMPLINKFLAGS= >> BVHTransform_c_mex.mki
echo EMC_COMPILER=msvc120>> BVHTransform_c_mex.mki
echo EMC_CONFIG=debug>> BVHTransform_c_mex.mki
"D:\Program Files\MATLAB\R2014a\bin\win64\gmake" -B -f BVHTransform_c_mex.mk
