set( HEADER_PATH ${CMAKE_SOURCE_DIR}/include/osgHaptics )

set ( osgHaptics_LIBS) 

include (FindOSGHelper)
include (Find3DTouch)


include_directories( 
    ${CMAKE_SOURCE_DIR}/include 
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
    ${3DTOUCH_INCLUDE}
    ${3DTOUCH_INCLUDE_UTS}
    )

if(WIN32)
  add_definitions( -DOSGHAPTICS_EXPORTS )
endif(WIN32)


set(TARGET_SRC
    BBoxVisitor.cpp
    ContactState.cpp
    ForceEffect.cpp
    ForceOperator.cpp
    HapticDevice.cpp
    HapticRenderBin.cpp
    HapticRenderLeaf.cpp
    HapticRenderPrepareVisitor.cpp
    HapticRootNode.cpp
    HapticSpringNode.cpp
    HashedGridDrawable.cpp
    Material.cpp
    osgHaptics.cpp
    ShapeComposite.cpp
    Shape.cpp
    SpringForceOperator.cpp
    TouchModel.cpp
    TriangleExtractor.cpp
    Version.cpp
    VibrationForceOperator.cpp
   )


set(TARGET_H
    ${HEADER_PATH}/BBoxVisitor.h
    ${HEADER_PATH}/ContactEventHandler.h
    ${HEADER_PATH}/ContactState.h
    ${HEADER_PATH}/export.h
    ${HEADER_PATH}/ForceEffect.h
    ${HEADER_PATH}/ForceOperator.h
    ${HEADER_PATH}/HapticDevice.h
    ${HEADER_PATH}/HapticRenderBin.h
    ${HEADER_PATH}/HapticRenderLeaf.h
    ${HEADER_PATH}/HapticRenderPrepareVisitor.h
    ${HEADER_PATH}/HapticRootNode.h
    ${HEADER_PATH}/HapticSpringNode.h
    ${HEADER_PATH}/HashedGridDrawable.h
    ${HEADER_PATH}/HashedGrid.h
    ${HEADER_PATH}/Material.h
    ${HEADER_PATH}/MonoCullCallback.h
    ${HEADER_PATH}/osgHaptics.h
    ${HEADER_PATH}/RenderTriangleOperator.h
    ${HEADER_PATH}/ShapeComposite.h
    ${HEADER_PATH}/Shape.h
    ${HEADER_PATH}/SpringForceOperator.h
    ${HEADER_PATH}/TouchModel.h
    ${HEADER_PATH}/TriangleExtractor.h
    ${HEADER_PATH}/types.h
    ${HEADER_PATH}/UpdateDeviceCallback.h
    ${HEADER_PATH}/Version.h
   )


add_library( osgHaptics SHARED ${TARGET_SRC} ${TARGET_H} )

target_link_libraries( osgHaptics osgSensor ${OPENSCENEGRAPH_LIBRARIES} ${3DTOUCH_LIBS} ${OPENGL_LIBRARIES} )

#SET(INSTALL_INCDIR include)
#SET(INSTALL_BINDIR bin)
#IF(WIN32)
#    SET(INSTALL_LIBDIR bin)
#    SET(INSTALL_ARCHIVEDIR lib)
#ELSE()
#    SET(INSTALL_LIBDIR lib${LIB_POSTFIX})
#    SET(INSTALL_ARCHIVEDIR lib${LIB_POSTFIX})
#ENDIF()
#
#
#INSTALL(
#        TARGETS ${TARGET_NAME}
#        RUNTIME DESTINATION ${INSTALL_BINDIR}
#        LIBRARY DESTINATION ${INSTALL_LIBDIR}
#        ARCHIVE DESTINATION ${INSTALL_ARCHIVEDIR}
#    )
#
#INSTALL(
#        FILES       ${PUBLIC_HEADERS}
#        DESTINATION ${INSTALL_INCDIR}/${TARGET_NAME}
#    )
#
