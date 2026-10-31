# Public headers
set(SHARED_INCLUDEDIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)

# Here puts required headers
set(SHARED_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/xynova/base.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xynova/data_types.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xynova/exception.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xynova/force_control.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xynova/model.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xynova/motion_control_rt.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xynova/planner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xynova/robot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xynova/servo.h
    ${CMAKE_CURRENT_SOURCE_DIR}/xynova/utility.h
    )

# upgrade header
set(UPGRADE_SHARED_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/xynova/upgrade.h
)

set(SHARED_HEADERS ${SHARED_HEADERS} CACHE INTERNAL "Shared headers with examples")
set(UPGRADE_SHARED_HEADERS ${UPGRADE_SHARED_HEADERS} CACHE INTERNAL "Upgrade shared headers with examples")