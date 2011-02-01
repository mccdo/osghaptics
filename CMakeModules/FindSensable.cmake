find_path( SENSABLE_INCLUDE HL/hl.h
           HINTS
           "$ENV{SENSABLE_DIR}"
           PATH_SUFFIXES include
           PATHS
           /usr
           /usr/local )

find_path( SENSABLE_INCLUDE_UTS HLU/hlu.h
           HINTS
           "$ENV{SENSABLE_DIR}"
           PATH_SUFFIXES include/utilities
           PATHS
           /usr
           /usr/local )



