find_path( 3DTOUCH_INCLUDE HL/hl.h
           HINTS
           "$ENV{3DTOUCH_BASE}"
           PATH_SUFFIXES include
           PATHS
           /usr
           /usr/local )

find_path( 3DTOUCH_INCLUDE_UTS HLU/hlu.h
           HINTS
           "$ENV{3DTOUCH_BASE}"
           PATH_SUFFIXES 
            utilities/include
           PATHS
            /usr
            /usr/local )

find_library( 3DTOUCH_HL_LIB
                NAMES hl 
                HINTS
                "$ENV{3DTOUCH_BASE}"
                PATH_SUFFIXES 
                    lib/win32
                    /usr
                    /usr/local
            )

find_library( 3DTOUCH_HD_LIB
                NAMES hd 
                HINTS
                "$ENV{3DTOUCH_BASE}"
                PATH_SUFFIXES 
                    lib/win32
                    /usr
                    /usr/local
            )

find_library( 3DTOUCH_HLU_LIB
                NAMES hlu 
                HINTS
                "$ENV{3DTOUCH_BASE}"
                PATH_SUFFIXES 
                    utilities/lib/win32/ReleaseAcademicEdition
                    /usr
                    /usr/local
            )

find_library( 3DTOUCH_HDU_LIB
                NAMES hdu 
                HINTS
                "$ENV{3DTOUCH_BASE}"
                PATH_SUFFIXES 
                    utilities/lib/win32/ReleaseAcademicEdition
                    /usr
                    /usr/local
            )

set(3DTOUCH_LIBS 
        ${3DTOUCH_HL_LIB} 
        ${3DTOUCH_HD_LIB} 
        ${3DTOUCH_HLU_LIB} 
        ${3DTOUCH_HDU_LIB} 
                 )



#message ( "3DTOUCH_INCLUDE is ${3DTOUCH_INCLUDE}" )
#message ( "3DTOUCH_INCLUDE_UTS is ${3DTOUCH_INCLUDE_UTS}" )
#message ( "3DTOUCH_LIBS is ${3DTOUCH_LIBS}" )
