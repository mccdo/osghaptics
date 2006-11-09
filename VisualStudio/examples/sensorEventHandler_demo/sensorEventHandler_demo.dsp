# Microsoft Developer Studio Project File - Name="sensorEventHandler_demo" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=sensorEventHandler_demo - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "sensorEventHandler_demo.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "sensorEventHandler_demo.mak" CFG="sensorEventHandler_demo - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "sensorEventHandler_demo - Win32 Debug" (based on "Win32 (x86) Console Application")
!MESSAGE "sensorEventHandler_demo - Win32 Release" (based on "Win32 (x86) Console Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "sensorEventHandler_demo - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "..\..\..\bin"
# PROP BASE Intermediate_Dir "$(ProjectName)_debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\..\bin"
# PROP Intermediate_Dir "$(ProjectName)_debug"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /I "..\..\include" /I ""$(3DTOUCH_BASE)\include"" /I ""$(3DTOUCH_BASE)\utilities\include"" /ZI /W3 /Od /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /Gm /GR PRECOMP_VC7_TOBEREMOVED /GZ /c /GX 
# ADD CPP /nologo /MDd /I "..\..\include" /I ""$(3DTOUCH_BASE)\include"" /I ""$(3DTOUCH_BASE)\utilities\include"" /ZI /W3 /Od /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /Gm /GR PRECOMP_VC7_TOBEREMOVED /GZ /c /GX 
# ADD BASE MTL /nologo /win32 
# ADD MTL /nologo /win32 
# ADD BASE RSC /l 1033 
# ADD RSC /l 1033 
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo 
# ADD BSC32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib osgtextd.lib producerd.lib opengl32.lib osgd.lib osggad.lib osgproducerd.lib osgdbd.lib osgUtild.lib openthreadswin32d.lib hlud.lib hdud.lib hd.lib hl.lib /nologo /out:"..\..\..\bin\$(ProjectName)d.exe" /incremental:yes /libpath:"../../lib" /libpath:"$(3DTOUCH_BASE)\lib" /libpath:"$(3DTOUCH_BASE)\utilities\lib" /debug /pdb:"..\..\..\bin\osgHapticViewer.pdb" /pdbtype:sept /subsystem:console /machine:ix86 
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib osgtextd.lib producerd.lib opengl32.lib osgd.lib osggad.lib osgproducerd.lib osgdbd.lib osgUtild.lib openthreadswin32d.lib hlud.lib hdud.lib hd.lib hl.lib /nologo /out:"..\..\..\bin\$(ProjectName)d.exe" /incremental:yes /libpath:"../../lib" /libpath:"$(3DTOUCH_BASE)\lib" /libpath:"$(3DTOUCH_BASE)\utilities\lib" /debug /pdb:"..\..\..\bin\osgHapticViewer.pdb" /pdbtype:sept /subsystem:console /machine:ix86 

!ELSEIF  "$(CFG)" == "sensorEventHandler_demo - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "..\..\..\bin"
# PROP BASE Intermediate_Dir "$(ProjectName)_release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\..\bin"
# PROP Intermediate_Dir "$(ProjectName)_release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /I "..\..\include" /I "$(3DTOUCH_BASE)\include" /I "$(3DTOUCH_BASE)\utilities\include" /Zi /W3 /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /GR PRECOMP_VC7_TOBEREMOVED /c /GX 
# ADD CPP /nologo /MD /I "..\..\include" /I "$(3DTOUCH_BASE)\include" /I "$(3DTOUCH_BASE)\utilities\include" /Zi /W3 /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /GR PRECOMP_VC7_TOBEREMOVED /c /GX 
# ADD BASE MTL /nologo /win32 
# ADD MTL /nologo /win32 
# ADD BASE RSC /l 1033 
# ADD RSC /l 1033 
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo 
# ADD BSC32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib osgtext.lib producer.lib osg.lib osgga.lib opengl32.lib osgproducer.lib osgdb.lib osgUtil.lib hd.lib openthreadswin32.lib /nologo /out:"..\..\..\bin\$(ProjectName).exe" /incremental:no /libpath:"../../lib" /debug /pdbtype:sept /subsystem:console /opt:ref /opt:icf /machine:ix86 
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib osgtext.lib producer.lib osg.lib osgga.lib opengl32.lib osgproducer.lib osgdb.lib osgUtil.lib hd.lib openthreadswin32.lib /nologo /out:"..\..\..\bin\$(ProjectName).exe" /incremental:no /libpath:"../../lib" /debug /pdbtype:sept /subsystem:console /opt:ref /opt:icf /machine:ix86 

!ENDIF

# Begin Target

# Name "sensorEventHandler_demo - Win32 Debug"
# Name "sensorEventHandler_demo - Win32 Release"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;def;odl;idl;hpj;bat;asm;asmx"
# Begin Source File

SOURCE=..\..\..\examples\sensoreventhandler_demo\sensorEventHandler_demo.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\..\examples\material_demo\HapticMaterialVisitor.h
# End Source File
# End Group
# End Target
# End Project

