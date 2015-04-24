@echo off
set MATLAB=D:\Program Files\MATLAB\R2014a
set MATLAB_ARCH=win64
set MATLAB_BIN="D:\Program Files\MATLAB\R2014a\bin"
set ENTRYPOINT=mexFunction
set OUTDIR=.\
set LIB_NAME=testC_mex
set MEX_NAME=testC_mex
set MEX_EXT=.mexw64
call mexopts.bat
echo # Make settings for testC > testC_mex.mki
echo COMPILER=%COMPILER%>> testC_mex.mki
echo COMPFLAGS=%COMPFLAGS%>> testC_mex.mki
echo OPTIMFLAGS=%OPTIMFLAGS%>> testC_mex.mki
echo DEBUGFLAGS=%DEBUGFLAGS%>> testC_mex.mki
echo LINKER=%LINKER%>> testC_mex.mki
echo LINKFLAGS=%LINKFLAGS%>> testC_mex.mki
echo LINKOPTIMFLAGS=%LINKOPTIMFLAGS%>> testC_mex.mki
echo LINKDEBUGFLAGS=%LINKDEBUGFLAGS%>> testC_mex.mki
echo MATLAB_ARCH=%MATLAB_ARCH%>> testC_mex.mki
echo BORLAND=%BORLAND%>> testC_mex.mki
echo OMPFLAGS= >> testC_mex.mki
echo OMPLINKFLAGS= >> testC_mex.mki
echo EMC_COMPILER=msvc120>> testC_mex.mki
echo EMC_CONFIG=optim>> testC_mex.mki
"D:\Program Files\MATLAB\R2014a\bin\win64\gmake" -B -f testC_mex.mk
