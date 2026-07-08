cmake_minimum_required(VERSION 3.8)
project(k1muse_voice_intent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Conditional real LLM client ──
option(K1MUSE_ENABLE_REAL_LLM_CLIENT
  "Build the real llama-server HTTP client" ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(k1muse_common REQUIRED)
find_package(k1muse_voice_msgs REQUIRED)

if(K1MUSE_ENABLE_REAL_LLM_CLIENT)
  find_package(CURL REQUIRED)
  find_package(nlohmann_json REQUIRED)
  add_compile_definitions(K1MUSE_ENABLE_REAL_LLM_CLIENT)
endif()

# ── Core library (pure C++, no ROS dependencies in most files) ──
set(VOICE_INTENT_LIB_SOURCES
  src/asr_text_cleaner.cpp
  src/fast_intent.cpp
  src/mock_llm_intent_client.cpp
  src/llm_response_validator.cpp
  src/llm_intent_mapper.cpp
  src/intent_router.cpp
)

# Real LLM sources — only compiled when the option is ON.
if(K1MUSE_ENABLE_REAL_LLM_CLIENT)
  list(APPEND VOICE_INTENT_LIB_SOURCES
    src/llama_server_client.cpp
    src/real_llm_intent_client.cpp
  )
endif()

add_library(${PROJECT_NAME} SHARED ${VOICE_INTENT_LIB_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

if(K1MUSE_ENABLE_REAL_LLM_CLIENT)
  target_link_libraries(${PROJECT_NAME}
    CURL::libcurl
    nlohmann_json::nlohmann_json
  )
endif()

# ── ROS node library ──
add_library(${PROJECT_NAME}_node SHARED src/intent_node.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  k1muse_common
  k1muse_voice_msgs
)

# ── Executable ──
add_executable(intent_node_exe src/main.cpp)
target_link_libraries(intent_node_exe PRIVATE ${PROJECT_NAME}_node)
set_target_properties(intent_node_exe PROPERTIES OUTPUT_NAME intent_node)

# ── Install ──
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS intent_node_exe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Tests ──
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_asr_text_cleaner test/test_asr_text_cleaner.cpp)
  target_link_libraries(test_asr_text_cleaner ${PROJECT_NAME})

  ament_add_gtest(test_fast_intent test/test_fast_intent.cpp)
  target_link_libraries(test_fast_intent ${PROJECT_NAME})

  ament_add_gtest(test_intent_router test/test_intent_router.cpp)
  target_link_libraries(test_intent_router ${PROJECT_NAME})

  ament_add_gtest(test_intent_node test/test_intent_node.cpp)
  target_link_libraries(test_intent_node ${PROJECT_NAME}_node)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake rclcpp k1muse_common k1muse_voice_msgs)

ament_package()
