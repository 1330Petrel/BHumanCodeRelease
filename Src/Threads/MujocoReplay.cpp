/**
 * @file Threads/MujocoReplay.cpp
 *
 * This file implements the execution unit for the MuJoCo replay thread.  It also
 * prints the per-frame smoke summary (image size, timestamp, camera parameters,
 * luma statistics) and writes one BallPercept JSON object per frame when the
 * scenario provides BallPercept.
 *
 * @author robocup_mujoco integration
 */

#include "MujocoReplay.h"

#include "Framework/Blackboard.h"
#include "Framework/ModuleGraphRunner.h"
#include "Modules/Infrastructure/MujocoReplayProvider/MujocoReplayProvider.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"

#include <fstream>
#include <iostream>

REGISTER_EXECUTION_UNIT(MujocoReplay)

std::string MujocoReplay::outputPath;
std::string MujocoReplay::summaryPath;
std::atomic<unsigned> MujocoReplay::written{0};

bool MujocoReplay::beforeFrame()
{
  // One-shot diagnosis of what the graph actually produced.  The thread-local
  // ModuleGraphRunner cannot be queried from here (its instance lives in the
  // thread that constructed the container), so the blackboard is used instead.
  static bool diagnosed = false;
  if(!diagnosed)
  {
    diagnosed = true;
    Blackboard& blackboard = Blackboard::getInstance();
    std::cout << "[MujocoReplay] blackboard: CameraImage=" << (blackboard.exists("CameraImage") ? 1 : 0)
              << " CameraInfo=" << (blackboard.exists("CameraInfo") ? 1 : 0)
              << " CameraMatrix=" << (blackboard.exists("CameraMatrix") ? 1 : 0)
              << " FrameInfo=" << (blackboard.exists("FrameInfo") ? 1 : 0)
              << " ECImage=" << (blackboard.exists("ECImage") ? 1 : 0)
              << " provider totalFrames=" << MujocoReplayProvider::totalFrames() << std::endl;
  }
  return MujocoReplayProvider::isFrameDataComplete();
}

bool MujocoReplay::afterFrame()
{
  if(MujocoReplayProvider::hasFailed())
    return false;
  Blackboard& blackboard = Blackboard::getInstance();
  const bool hasImage = blackboard.exists("CameraImage");

  if(!hasImage)
  {
    // Only reachable if the module graph did not create MujocoReplayProvider.
    std::cout << "[MujocoReplay] no CameraImage in the blackboard (graph iteration "
              << MujocoReplayProvider::framesProvided() << " frames so far)" << std::endl;
  }

  if(hasImage && blackboard.exists("CameraInfo"))
  {
    const CameraImage& image = static_cast<const CameraImage&>(blackboard["CameraImage"]);
    const CameraInfo& info = static_cast<const CameraInfo&>(blackboard["CameraInfo"]);

    // Mean luma over a sparse grid proves the native accessors really read the
    // YUYV payload that robocup_mujoco wrote.
    unsigned long long sum = 0;
    unsigned samples = 0;
    for(unsigned y = 0; y < image.height; y += 16)
      for(unsigned x = 0; x < image.width * 2; x += 16)
      {
        sum += image.getY(x, y);
        ++samples;
      }

    std::cout << "[MujocoReplay] frame " << MujocoReplayProvider::currentFrame()
              << " timestamp=" << (blackboard.exists("FrameInfo")
                                   ? static_cast<const FrameInfo&>(blackboard["FrameInfo"]).time
                                   : image.timestamp)
              << " pairs=" << image.width << "x" << image.height
              << " pixels=" << info.width << "x" << info.height
              << " fx=" << info.focalLength << " fy=" << info.focalLengthHeight
              << " cx=" << info.opticalCenter.x() << " cy=" << info.opticalCenter.y()
              << " openingAngle=" << info.openingAngleWidth << "/" << info.openingAngleHeight;
    if(blackboard.exists("CameraMatrix"))
    {
      const CameraMatrix& cameraMatrix = static_cast<const CameraMatrix&>(blackboard["CameraMatrix"]);
      std::cout << " camera_t=" << cameraMatrix.translation.x() << "," << cameraMatrix.translation.y()
                << "," << cameraMatrix.translation.z() << " valid=" << (cameraMatrix.isValid ? 1 : 0);
    }
    std::cout << " meanY=" << (samples ? static_cast<double>(sum) / samples : 0.0) << std::endl;

    // Race-free hand-off to the tool: it reads this file after the thread stopped.
    if(!summaryPath.empty())
    {
      std::ofstream summary(summaryPath);
      if(summary.is_open())
      {
        summary << "{\"framesProvided\":" << MujocoReplayProvider::framesProvided()
                << ",\"totalFrames\":" << MujocoReplayProvider::totalFrames()
                << ",\"lastFrame\":" << MujocoReplayProvider::currentFrame()
                << ",\"timestamp\":" << image.timestamp
                << ",\"width_pairs\":" << image.width
                << ",\"height\":" << image.height
                << ",\"width\":" << info.width
                << ",\"height_px\":" << info.height
                << ",\"fx\":" << info.focalLength
                << ",\"fy\":" << info.focalLengthHeight
                << ",\"cx\":" << info.opticalCenter.x()
                << ",\"cy\":" << info.opticalCenter.y()
                << ",\"meanY\":" << (samples ? static_cast<double>(sum) / samples : 0.0);
        if(blackboard.exists("CameraMatrix"))
        {
          const CameraMatrix& cameraMatrix = static_cast<const CameraMatrix&>(blackboard["CameraMatrix"]);
          summary << ",\"cameraTranslation_mm\":[" << cameraMatrix.translation.x() << ","
                  << cameraMatrix.translation.y() << "," << cameraMatrix.translation.z() << "]"
                  << ",\"cameraMatrixValid\":" << (cameraMatrix.isValid ? "true" : "false");
        }
        summary << "}" << std::endl;
      }
    }
  }

  if(!outputPath.empty() && blackboard.exists("BallPercept"))
  {
    const BallPercept& percept = static_cast<const BallPercept&>(blackboard["BallPercept"]);
    std::ofstream out(outputPath, writtenPercepts() == 0 ? std::ios::trunc : std::ios::app);
    if(out.is_open())
    {
      out << "{\"frame\":" << MujocoReplayProvider::currentFrame()
          << ",\"status\":\"" << (percept.status == BallPercept::seen ? "seen"
                                  : percept.status == BallPercept::guessed ? "guessed" : "notSeen") << "\""
          << ",\"positionInImage\":[" << percept.positionInImage.x() << "," << percept.positionInImage.y() << "]"
          << ",\"radiusInImage\":" << percept.radiusInImage
          << ",\"positionOnField\":[" << percept.positionOnField.x() << "," << percept.positionOnField.y() << "]"
          << ",\"radiusOnField\":" << percept.radiusOnField << "}\n";
      written.fetch_add(1, std::memory_order_release);
    }
  }

  MujocoReplayProvider::waitForFrameData();
  return false; // Self-clocked replay: do not wait for an external camera message.
}
