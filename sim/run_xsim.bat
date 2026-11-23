@echo off
REM Vivado simulator (xsim), Windows. Open the "Vivado <version> Tcl Shell"
REM (or a command prompt with Vivado\bin on PATH), cd to this folder, then:
REM   run_xsim.bat                 default model (vectors)
REM   run_xsim.bat vectors_paper   Table I word lengths of the paper
cd /d "%~dp0"
set VDIR=%1
if "%VDIR%"=="" set VDIR=vectors
cd %VDIR%
call xvlog --relax -d VEC_FLAT -i . ..\tb_mod_top.v ..\..\rtl\gmm_core.v ..\..\rtl\bin_morph3x3.v ..\..\rtl\morph_chain.v ..\..\rtl\axis_skid.v ..\..\rtl\ccl_single_pass.v ..\..\rtl\mod_axil_regs.v ..\..\rtl\mod_top.v > xvlog.log || goto :err
call xelab --relax -debug typical -s tb_mod_top_sim tb_mod_top > xelab.log || goto :err
call xsim tb_mod_top_sim -R > xsim_run.log
findstr /C:"TEST" /C:"ERROR" /C:"done" /C:"mismatches" xsim_run.log
cd ..
goto :eof
:err
echo Compilation failed - see xvlog.log / xelab.log in %VDIR%
cd ..
