protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${CMAKE_CURRENT_SOURCE_DIR} state_estimator.proto)
add_library(state_estimator SHARED f_state_estimator.cpp ${CHANS} ${PROTO_SRCS})

target_include_directories(state_estimator PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(state_estimator PUBLIC ${PROJECT_SOURCE_DIR}/channels)
install(TARGETS state_estimator DESTINATION lib)
