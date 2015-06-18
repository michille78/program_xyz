@echo off
set MATLAB=D:\Program Files\MATLAB\R2014a
set MATLAB_ARCH=win64
set MATLAB_BIN="D:\Program Files\MATLAB\R2014a\bin"
set ENTRYPOINT=mexFunction
set OUTDIR=.\
set LIB_NAME=GetINSCompensateFromVNS_mex
set MEX_NAME=GetINSCompensateFromVNS_mex
set MEX_EXT=.mexw64
call mexopts.bat
echo # Make settings for GetINSCompensateFromVNS > GetINSCompensateFromVNS_mex.mki
echo COMPILER=%COMPILER%>> GetINSCompensateFromVNS_mex.mki
echo COMPFLAGS=%COMPFLAGS%>> GetINSCompensateFromVNS_mex.mki
echo OPTIMFLAGS=%OPTIMFLAGS%>> GetINSCompensateFromVNS_mex.mki
echo DEBUGFLAGS=%DEBUGFLAGS%>> GetINSCompensateFromVNS_mex.mki
echo LINKER=%LINKER%>> GetINSCompensateFromVNS_mex.mki
echo LINKFLAGS=%LINKFLAGS%>> GetINSCompensateFromVNS_mex.mki
echo LINKOPTIMFLAGS=%LINKOPTIMFLAGS%>> GetINSCompensateFromVNS_mex.mki
echo LINKDEBUGFLAGS=%LINKDEBUGFLAGS%>> GetINSCompensateFromVNS_mex.mki
echo MATLAB_ARCH=%MATLAB_ARCH%>> GetINSCompensateFromVNS_mex.mki
echo BORLAND=%BORLAND%>> GetINSCompensateFromVNS_mex.mki
echo OMPFLAGS= >> GetINSCompensateFromVNS_mex.mki
echo OMPLINKFLAGS= >> GetINSCompensateFromVNS_mex.mki
echo EMC_COMPILER=msvc100>> GetINSCompensateFromVNS_mex.mki
echo EMC_CONFIG=optim>> GetINSCompensateFromVNS_mex.mki
"D:\Program Files\MATLAB\R2014a\bin\win64\gmake" -B -f GetINSCompensateFromVNS_mex.mk
