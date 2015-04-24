@echo off
set MATLAB=D:\Program Files\MATLAB\R2014a
set MATLAB_ARCH=win64
set MATLAB_BIN="D:\Program Files\MATLAB\R2014a\bin"
set ENTRYPOINT=mexFunction
set OUTDIR=.\
set LIB_NAME=pythagoras_mex
set MEX_NAME=pythagoras_mex
set MEX_EXT=.mexw64
call mexopts.bat
echo # Make settings for pythagoras > pythagoras_mex.mki
echo COMPILER=%COMPILER%>> pythagoras_mex.mki
echo COMPFLAGS=%COMPFLAGS%>> pythagoras_mex.mki
echo OPTIMFLAGS=%OPTIMFLAGS%>> pythagoras_mex.mki
echo DEBUGFLAGS=%DEBUGFLAGS%>> pythagoras_mex.mki
echo LINKER=%LINKER%>> pythagoras_mex.mki
echo LINKFLAGS=%LINKFLAGS%>> pythagoras_mex.mki
echo LINKOPTIMFLAGS=%LINKOPTIMFLAGS%>> pythagoras_mex.mki
echo LINKDEBUGFLAGS=%LINKDEBUGFLAGS%>> pythagoras_mex.mki
echo MATLAB_ARCH=%MATLAB_ARCH%>> pythagoras_mex.mki
echo BORLAND=%BORLAND%>> pythagoras_mex.mki
echo OMPFLAGS= >> pythagoras_mex.mki
echo OMPLINKFLAGS= >> pythagoras_mex.mki
echo EMC_COMPILER=msvc120>> pythagoras_mex.mki
echo EMC_CONFIG=optim>> pythagoras_mex.mki
"D:\Program Files\MATLAB\R2014a\bin\win64\gmake" -B -f pythagoras_mex.mk
