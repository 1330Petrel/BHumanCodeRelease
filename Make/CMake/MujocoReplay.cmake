set(MUJOCO_REPLAY_ROOT_DIR "${BHUMAN_PREFIX}/Src/Apps/MujocoReplay")
set(MUJOCO_REPLAY_OUTPUT_DIR "${OUTPUT_PREFIX}/Build/${PLATFORM}/MujocoReplay/$<CONFIG>")

file(GLOB_RECURSE MUJOCO_REPLAY_SOURCES CONFIGURE_DEPENDS
    "${MUJOCO_REPLAY_ROOT_DIR}/*.cpp" "${MUJOCO_REPLAY_ROOT_DIR}/*.h")

add_executable(MujocoReplay EXCLUDE_FROM_ALL ${MUJOCO_REPLAY_SOURCES})

set_property(TARGET MujocoReplay PROPERTY RUNTIME_OUTPUT_DIRECTORY "${MUJOCO_REPLAY_OUTPUT_DIR}")
set_property(TARGET MujocoReplay PROPERTY FOLDER Apps)

target_include_directories(MujocoReplay PRIVATE "${MUJOCO_REPLAY_ROOT_DIR}")

# B-Human carries all modules, including MujocoReplayProvider.
target_link_libraries(MujocoReplay PRIVATE B-Human)
target_link_libraries(MujocoReplay PRIVATE Framework)
target_link_libraries(MujocoReplay PRIVATE Streaming)
target_link_libraries(MujocoReplay PRIVATE Flags::Default)

source_group(TREE "${MUJOCO_REPLAY_ROOT_DIR}" FILES ${MUJOCO_REPLAY_SOURCES})
