# Microsoft Developer Studio Project File - Name="osgSensor" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Dynamic-Link Library" 0x0102

CFG=osgSensor - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "osgSensor.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "osgSensor.mak" CFG="osgSensor - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "osgSensor - Win32 Debug" (based on "Win32 (x86) Dynamic-Link Library")
!MESSAGE "osgSensor - Win32 Release" (based on "Win32 (x86) Dynamic-Link Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "osgSensor - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "../../bin"
# PROP BASE Intermediate_Dir "$(ProjectName)_Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "../../bin"
# PROP Intermediate_Dir "$(ProjectName)_Debug"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /I "../../include" /ZI /W3 /Od /G6 /D "_DEBUG" /D "_WINDOWS" /D "_USRDLL" /D "EXPORT_OSGSENSOR" /D "WIN32" /D "_MBCS" /GR /YX /Fp".\Debug/$(ProjectName).pch" /Fo".\Debug/" /Fd".\Debug/" /FR /GZ /c /Zm200/GX 
# ADD CPP /nologo /MDd /I "../../include" /ZI /W3 /Od /G6 /D "_DEBUG" /D "_WINDOWS" /D "_USRDLL" /D "EXPORT_OSGSENSOR" /D "WIN32" /D "_MBCS" /GR /YX /Fp".\Debug/$(ProjectName).pch" /Fo".\Debug/" /Fd".\Debug/" /FR /GZ /c /Zm200/GX 
# ADD BASE MTL /nologo /D"_DEBUG" /mktyplib203 /tlb"..\..\lib\osgSensor.tlb" /win32 
# ADD MTL /nologo /D"_DEBUG" /mktyplib203 /tlb"..\..\lib\osgSensor.tlb" /win32 
# ADD BASE RSC /l 1053 /d "_DEBUG" 
# ADD RSC /l 1053 /d "_DEBUG" 
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo 
# ADD BSC32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib  osgd.lib osgGAd.lib openthreadswin32d.lib /nologo /dll /out:"..\..\bin\$(ProjectName)d.dll" /incremental:yes /nodefaultlib:"msvcrt" /debug /pdb:"$(ProjectName)_Debug\$(ProjectName)d.pdb" /pdbtype:sept /subsystem:windows /implib:"../../lib/$(ProjectName)d.lib" /machine:ix86 /MACHINE:I386
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib  osgd.lib osgGAd.lib openthreadswin32d.lib /nologo /dll /out:"..\..\bin\$(ProjectName)d.dll" /incremental:yes /nodefaultlib:"msvcrt" /debug /pdb:"$(ProjectName)_Debug\$(ProjectName)d.pdb" /pdbtype:sept /subsystem:windows /implib:"../../lib/$(ProjectName)d.lib" /machine:ix86 /MACHINE:I386

!ELSEIF  "$(CFG)" == "osgSensor - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "../../bin"
# PROP BASE Intermediate_Dir "$(ProjectName)_Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "../../bin"
# PROP Intermediate_Dir "$(ProjectName)_Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /I "../../include" /W3 /Ob1 /D "NDEBUG" /D "_WINDOWS" /D "_USRDLL" /D "EXPORT_OSGSENSOR" /D "WIN32" /D "_MBCS" /GF /Gy /GR /YX /Fp".\Release/$(ProjectName).pch" /Fo".\Release/" /Fd".\Release/" /c /Zm200/GX 
# ADD CPP /nologo /MD /I "../../include" /W3 /Ob1 /D "NDEBUG" /D "_WINDOWS" /D "_USRDLL" /D "EXPORT_OSGSENSOR" /D "WIN32" /D "_MBCS" /GF /Gy /GR /YX /Fp".\Release/$(ProjectName).pch" /Fo".\Release/" /Fd".\Release/" /c /Zm200/GX 
# ADD BASE MTL /nologo /D"NDEBUG" /mktyplib203 /tlb"..\..\lib\osgSensor.tlb" /win32 
# ADD MTL /nologo /D"NDEBUG" /mktyplib203 /tlb"..\..\lib\osgSensor.tlb" /win32 
# ADD BASE RSC /l 1053 /d "NDEBUG" 
# ADD RSC /l 1053 /d "NDEBUG" 
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo 
# ADD BSC32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib  osg.lib osgGA.lib openthreadswin32.lib /nologo /dll /out:"..\..\bin\$(ProjectName).dll" /incremental:no /nodefaultlib:"LIBCD" /pdb:"$(ProjectName)_Release\$(ProjectName).pdb" /pdbtype:sept /subsystem:windows /implib:"../../lib/$(ProjectName).lib" /MACHINE:I386
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib  osg.lib osgGA.lib openthreadswin32.lib /nologo /dll /out:"..\..\bin\$(ProjectName).dll" /incremental:no /nodefaultlib:"LIBCD" /pdb:"$(ProjectName)_Release\$(ProjectName).pdb" /pdbtype:sept /subsystem:windows /implib:"../../lib/$(ProjectName).lib" /MACHINE:I386

!ENDIF

# Begin Target

# Name "osgSensor - Win32 Debug"
# Name "osgSensor - Win32 Release"
# Begin Group "Source files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\src\osgSensor\Event.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgSensor\KeyboardSensor.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgSensor\OsgSensor.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgSensor\OsgSensorCallback.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgSensor\Sensor.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgSensor\SensorLink.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgSensor\SensorLinkCallback.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgSensor\SensorManipulator.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgSensor\SensorMgr.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgSensor\ThreadedSensor.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgSensor\TransformSensor.cpp
# End Source File
# Begin Source File

SOURCE=..\..\src\osgSensor\Visitors.cpp
# End Source File
# End Group
# Begin Group "Header files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\include\osgSensor\Event.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgSensor\export.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgSensor\KeyboardSensor.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgSensor\OsgSensor.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgSensor\OsgSensorCallback.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgSensor\Sensor.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgSensor\SensorLink.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgSensor\SensorLinkCallback.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgSensor\SensorManipulator.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgSensor\SensorMgr.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgSensor\ThreadedSensor.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgSensor\TransformSensor.h
# End Source File
# Begin Source File

SOURCE=..\..\include\osgSensor\Visitors.h
# End Source File
# End Group
# End Target
# End Project

