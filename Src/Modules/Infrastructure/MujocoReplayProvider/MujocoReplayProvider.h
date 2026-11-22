/**
 * @file MujocoReplayProvider.h
 *
 * This file declares a module that provides CameraImage, CameraInfo,
 * CameraMatrix and FrameInfo from a MuJoCo replay bundle written by
 * robocup_mujoco (see scripts/bhuman_frame_provider.py).
 *
 * The module is the simulated counterpart of X5CameraProvider/RealSenseProvider:
 * instead of grabbing from a camera service it replays YUYV frames listed in
 * manifest.tsv, together with the calibrated camera pose of the simulator, so
 * that RobotBallPerceptor and the rest of the vision pipeline can run offline.
 *
 * @author robocup_mujoco integration
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"

#include <atomic>
#include <string>
#include <vector>

MODULE(MujocoReplayProvider,
{,
  PROVIDES_WITHOUT_MODIFY(CameraImage),
  PROVIDES(CameraInfo),
  PROVIDES(CameraMatrix),
  PROVIDES(FrameInfo),
  LOADS_PARAMETERS(
  {,
    (std::string) manifestPath, /**< Path to manifest.tsv; empty means "set via setManifest()". */
    (bool) loop, /**< Restart from the first frame after the last one. */
    (int) frameDelay, /**< Milliseconds to pause in waitForFrameData() to emulate the camera rate. */
    (bool) provideCameraMatrix, /**< Provide CameraMatrix directly instead of CameraMatrixProvider. */
  }),
});

class MujocoReplayProvider : public MujocoReplayProviderBase
{
private:
  /** One row of manifest.tsv. */
  struct Record
  {
    unsigned timestamp = 0;
    int widthPairs = 0;
    int height = 0;
    float fx = 0.f;
    float fy = 0.f;
    float cx = 0.f;
    float cy = 0.f;
    float rotation[3][3] = {{1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {0.f, 0.f, 1.f}};
    float translation[3] = {0.f, 0.f, 0.f}; /**< camera centre in mm, robot frame */
    float ballRadius = 95.f;
    std::string imageFile;
  };

  // The provider lives in the module thread while the tool polls it from the main
  // thread, so the published state is atomic.
  static std::atomic<MujocoReplayProvider*> theInstance; /**< The instance in this thread or nullptr. */
  static std::string manifestPathOverride; /**< Manifest path set via setManifest() before the graph starts. */

  std::vector<Record> records; /**< All frames of the replay. */
  std::vector<unsigned char> buffer; /**< Backing storage the provided CameraImage references. */
  size_t index = 0; /**< Index of the frame currently provided. */
  size_t loadedIndex = static_cast<size_t>(-1); /**< Index whose pixels are in buffer. */
  bool provided = false; /**< Was the frame at index already handed to the graph? */
  std::atomic<unsigned> providedFrames{0}; /**< How many frames have been provided so far. */
  bool failed = false; /**< Was loading the manifest impossible? */

  /** Load manifest.tsv. */
  bool loadManifest();

  /** Read the YUYV payload of a record into the buffer. */
  bool loadImage(const Record& record);

public:
  MujocoReplayProvider();
  ~MujocoReplayProvider();

  /** The live instance in the module thread (nullptr before it is created). */
  static MujocoReplayProvider* instance();

  /**
   * Override the manifest path before the module graph is created.
   * Used by the MujocoReplay command line tool.
   */
  static void setManifest(const std::string& path) {manifestPathOverride = path;}

  /** Are there frames left to replay? */
  static bool hasMoreFrames();

  /** The index of the frame that is currently provided. */
  static size_t currentFrame();

  /** Total number of frames in the manifest. */
  static size_t totalFrames();

  /** How many frames were handed to the module graph so far. */
  static unsigned framesProvided();

  /** Is the replay finished (all frames provided and no loop requested)? */
  static bool finished();

  void update(CameraImage& theCameraImage) override;
  void update(CameraInfo& theCameraInfo) override;
  void update(CameraMatrix& theCameraMatrix) override;
  void update(FrameInfo& theFrameInfo) override;

  /** The method returns whether a new image is available. */
  static bool isFrameDataComplete();

  /** The method waits for a new image (emulates the camera frame rate). */
  static void waitForFrameData();
};
