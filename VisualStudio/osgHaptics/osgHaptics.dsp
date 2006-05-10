# Microsoft Developer Studio Project File - Name="osgHaptics" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Dynamic-Link Library" 0x0102

CFG=osgHaptics - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "osgHaptics.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "osgHaptics.mak" CFG="osgHaptics - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "osgHaptics - Win32 Debug" (based on "Win32 (x86) Dynamic-Link Library")
!MESSAGE "osgHaptics - Win32 Release" (based on "Win32 (x86) Dynamic-Link Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "osgHaptics - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "..\..\bin"
# PROP BASE Intermediate_Dir "$(ProjectName)_debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\bin"
# PROP Intermediate_Dir "$(ProjectName)_debug"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /ZI /W3 /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_USRDLL" /D "OSGHAPTICS_EXPORTS" /D "_MBCS" /Gm /GR PRECOMP_VC7_TOBEREMOVED /Fp"$(IntDir)/$(TargetName)d.pch" /FR /GZ /c /GX 
# ADD CPP /nologo /MDd /ZI /W3 /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_USRDLL" /D "OSGHAPTICS_EXPORTS" /D "_MBCS" /Gm /GR PRECOMP_VC7_TOBEREMOVED /Fp"$(IntDir)/$(TargetName)d.pch" /FR /GZ /c /GX 
# ADD BASE MTL /nologo /win32 
# ADD MTL /nologo /win32 
# ADD BASE RSC /l 1033 
# ADD RSC /l 1033 
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo 
# ADD BSC32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib osgproducerd.lib  osgUtild.lib openthreadswin32d.lib hlud.lib hdud.lib hd.lib hl.lib osgd.lib opengl32.lib /nologo /dll /out:"..\..\bin\$(ProjectName)d.dll" /incremental:yes /debug /pdb:"$(ProjectName)_debug\$(ProjectName)d.pdb" /pdbtype:sept /subsystem:windows /implib:"../../lib/$(ProjectName)d.lib" 
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib osgproducerd.lib  osgUtild.lib openthreadswin32d.lib hlud.lib hdud.lib hd.lib hl.lib osgd.lib opengl32.lib /nologo /dll /out:"..\..\bin\$(ProjectName)d.dll" /incremental:yes /debug /pdb:"$(ProjectName)_debug\$(ProjectName)d.pdb" /pdbtype:sept /subsystem:windows /implib:"../../lib/$(ProjectName)d.lib" 

!ELSEIF  "$(CFG)" == "osgHaptics - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "..\..\bin"
# PROP BASE Intermediate_Dir "$(ProjectName)_Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\bin"
# PROP Intermediate_Dir "$(ProjectName)_Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /Zi /W3 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_USRDLL" /D "OSGHAPTICS_EXPORTS" /D "_MBCS" /GR /YX /c /GX 
# ADD CPP /nologo /MD /Zi /W3 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_USRDLL" /D "OSGHAPTICS_EXPORTS" /D "_MBCS" /GR /YX /c /GX 
# ADD BASE MTL /nologo /win32 
# ADD MTL /nologo /win32 
# ADD BASE RSC /l 1033 
# ADD RSC /l 1033 
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo 
# ADD BSC32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib osgproducer.lib osgUtil.lib openthreadswin32.lib hlu.lib hdu.lib hd.lib hl.lib osg.lib opengl32.lib /nologo /dll /out:"..\..\bin\$(ProjectName).dll" /incremental:no /libpath:"$(3DTOUCH_BASE)\lib" /libpath:"$(3DTOUCH_BASE)\utilities\lib" /debug /pdbtype:sept /subsystem:windows /opt:ref /opt:icf /implib:"../../lib/$(ProjectName).lib" /machine:ix86 
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib osgproducer.lib osgUtil.lib openthreadswin32.lib hlu.lib hdu.lib hd.lib hl.lib osg.lib opengl32.lib /nologo /dll /out:"..\..\bin\$(ProjectName).dll" /incremental:no /libpath:"$(3DTOUCH_BASE)\lib" /libpath:"$(3DTOUCH_BASE)\utilities\lib" /debug /pdbtype:sept /subsystem:windows /opt:ref /opt:icf /implib:"../../lib/$(ProjectName).lib" /machine:ix86 

!ENDIF

# Begin Target

# Name "osgHaptics - Win32 Debug"
# Name "osgHaptics - Win32 Release"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;def;odl;idl;hpj;bat;asm;asmx"
# Begin Source File

SOURCE=..\..\src\osgHaptics\ContactState.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgHaptics\ForceEffect.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgHaptics\ForceOperator.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgHaptics\HapticDevice.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgHaptics\HapticRenderBin.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgHaptics\HapticRenderLeaf.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgHaptics\HapticRenderPrepareVisitor.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgHaptics\HapticRootNode.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgHaptics\HapticSpringNode.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgHaptics\Material.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgHaptics\osgHaptics.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgHaptics\Shape.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgHaptics\ShapeComposite.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgHaptics\SpringForceOperator.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgHaptics\TouchModel.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgHaptics\VibrationForceOperator.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl;inc;xsd"
# Begin Source File

SOURCE=..\..\include\osgHaptics\ContactEventHandler.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\ContactState.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\EventHandler.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\export.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\ForceEffect.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\ForceOperator.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\HapticDevice.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\HapticRenderBin.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\HapticRenderLeaf.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\HapticRenderPrepareVisitor.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\HapticRootNode.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\HapticShapeGroup.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\HapticSpringNode.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\Material.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\MonoCullCallback.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\osgHaptics.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\RenderTriangleOperator.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\Shape.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\ShapeComposite.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\SpringForceOperator.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\TouchModel.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\types.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\UpdateDeviceCallback.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgHaptics\VibrationForceOperator.h
# End Source File
# End Group
# End Target
# End Project

