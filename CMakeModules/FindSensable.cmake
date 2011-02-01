find_path( SENSABLE_INCLUDE HL/hl.h
           HINTS
           "$ENV{3DTOUCH_BASE}"
           PATH_SUFFIXES include
           PATHS
           /usr
           /usr/local )

find_path( SENSABLE_INCLUDE_UTS HLU/hlu.h
           HINTS
           "$ENV{3DTOUCH_BASE}"
           PATH_SUFFIXES utilities/include
           PATHS
           /usr
           /usr/local )



#message ( "SENSABLE_INCLUDE is ${SENSABLE_INCLUDE}" )
#message ( "SENSABLE_INCLUDE_UTS is ${SENSABLE_INCLUDE_UTS}" )
