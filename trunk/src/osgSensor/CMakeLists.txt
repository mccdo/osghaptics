set( HEADER_PATH ${CMAKE_SOURCE_DIR}/include/osgSensor )

include_directories( 
    ${CMAKE_SOURCE_DIR}/include 
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
    )


set(TARGET_SRC
    KeyboardSensor.cpp
    OsgSensorCallback.cpp
    OsgSensor.cpp
    Sensor.cpp
    SensorEventHandler.cpp
    SensorLinkCallback.cpp
    SensorLink.cpp
    SensorManipulator.cpp
    SensorMgr.cpp
    ThreadedSensor.cpp
    TransformSensor.cpp
    Visitors.cpp
   )


set(TARGET_H
    ${HEADER_PATH}/bitoperators.h
    ${HEADER_PATH}/export.h
    ${HEADER_PATH}/KeyboardSensor.h
    ${HEADER_PATH}/OsgSensorCallback.h
    ${HEADER_PATH}/OsgSensor.h
    ${HEADER_PATH}/SensorEventHandler.h
    ${HEADER_PATH}/Sensor.h
    ${HEADER_PATH}/SensorLinkCallback.h
    ${HEADER_PATH}/SensorLink.h
    ${HEADER_PATH}/SensorManipulator.h
    ${HEADER_PATH}/SensorMgr.h
    ${HEADER_PATH}/StopThread.h
    ${HEADER_PATH}/ThreadedSensor.h
    ${HEADER_PATH}/TransformSensor.h
    ${HEADER_PATH}/Visitors.h
   )


set(TARGET_NAME osgSensor)
add_library( ${TARGET_NAME} SHARED ${TARGET_SRC} ${TARGET_H} )
target_link_libraries(${TARGET_NAME} ${osgSensor_LIBS} ${OPENSCENEGRAPH_LIBRARIES})


