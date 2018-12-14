add_library( rgbd_tutor_lib
    rgbdframe.cpp 
    parameter_reader.cpp 
    rgbd_reader.cpp
    orb.cpp 
    pnp.cpp 
    track.cpp 
    looper.cpp 
    pose_graph.cpp
    mapper.cpp
    ros_wrapper.cpp
)

target_link_libraries( rgbd_tutor_lib
    ${OpenCV_LIBRARIES}
    ${CHOLMOD_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/Thirdparty/orbslam_modified/lib/libORB_SLAM2.so
    ${g2o_libs}
    ${VTK_LIBRARIES}
    ${catkin_LIBRARIES}
    )
