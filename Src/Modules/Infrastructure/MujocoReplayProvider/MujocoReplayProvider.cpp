/**
 * @file MujocoReplayProvider.cpp
 *
 * This file implements a module that replays a MuJoCo camera bundle into the
 * B-Human vision pipeline.  See MujocoReplayProvider.h for the contract.
 *
 * The manifest consumed here is ``manifest.tsv`` written by robocup_mujoco:
 *
 *   # timestamp_ms width_pairs height fx fy cx cy r00 .. r22 tx ty tz ball_radius_mm yuyv_file
 *
 * ``width_pairs`` is the number of YUYV pixel pairs, i.e. exactly
 * ``CameraImage.width``; the pixel width in ``CameraInfo`` is twice that.
 *
 * @author robocup_mujoco integration
 */

#include "MujocoReplayProvider.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"
#include "Platform/Time.h"
#include "Math/RotationMatrix.h"
#include "Tools/Math/Transformation.h"
#include "Framework/Blackboard.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

MAKE_MODULE(MujocoReplayProvider);

std::atomic<MujocoReplayProvider*> MujocoReplayProvider::theInstance{nullptr};
std::string MujocoReplayProvider::manifestPathOverride;

/** Report a load failure both on stdout (headless tools have no debug channel)
 *  and via OUTPUT_ERROR (which goes to the framework's debug queue). */
static void reportFailure(const std::string& message)
{
  std::cerr << "[MujocoReplayProvider] " << message << std::endl;
  OUTPUT_ERROR(message.c_str());
}

MujocoReplayProvider* MujocoReplayProvider::instance()
{
  return theInstance.load(std::memory_order_acquire);
}

MujocoReplayProvider::MujocoReplayProvider()
{
  if(!loadManifest())
    failed = true;
  // RobotBallPerceptor selects its model in its constructor.
  if(Blackboard::getInstance().exists("CameraInfo"))
    update(static_cast<CameraInfo&>(Blackboard::getInstance()["CameraInfo"]));
  theInstance.store(this, std::memory_order_release);
}

MujocoReplayProvider::~MujocoReplayProvider()
{
  theInstance = nullptr;
}

bool MujocoReplayProvider::loadManifest()
{
  const std::string path = !manifestPathOverride.empty() ? manifestPathOverride : manifestPath;
  if(path.empty())
    return false;

  std::ifstream stream(path);
  if(!stream.is_open())
  {
    reportFailure("cannot open " + path);
    return false;
  }

  std::string line;
  const size_t base = path.find_last_of("/\\");
  const std::string directory = base == std::string::npos ? std::string() : path.substr(0, base + 1);
  while(std::getline(stream, line))
  {
    if(line.empty() || line[0] == '#')
      continue;
    std::istringstream fields(line);
    Record record;
    std::string imageFile;
    fields >> record.timestamp >> record.widthPairs >> record.height
           >> record.fx >> record.fy >> record.cx >> record.cy;
    for(int r = 0; r < 3; ++r)
      for(int c = 0; c < 3; ++c)
        fields >> record.rotation[r][c];
    fields >> record.translation[0] >> record.translation[1] >> record.translation[2]
           >> record.ballRadius >> imageFile;
    if(!fields)
    {
      reportFailure("malformed manifest line: " + line);
      return false;
    }
    record.imageFile = directory + imageFile;
    records.push_back(record);
  }
  if(records.empty())
  {
    reportFailure("no frames in " + path);
    return false;
  }
  return true;
}

bool MujocoReplayProvider::loadImage(const Record& record)
{
  std::ifstream stream(record.imageFile, std::ios::binary);
  if(!stream.is_open())
  {
    reportFailure("cannot open " + record.imageFile);
    return false;
  }
  const size_t expected = static_cast<size_t>(record.widthPairs) * record.height * 4; // YUYVPixel = {y0,u,y1,v}
  buffer.assign(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
  if(buffer.size() != expected)
  {
    OUTPUT_ERROR("MujocoReplayProvider: " << record.imageFile.c_str() << " has "
                 << static_cast<unsigned>(buffer.size()) << " bytes, expected "
                 << static_cast<unsigned>(expected));
    std::cerr << "[MujocoReplayProvider] " << record.imageFile << " has " << buffer.size()
              << " bytes, expected " << expected << std::endl;
    return false;
  }
  return true;
}

void MujocoReplayProvider::update(CameraImage& theCameraImage)
{
  if(failed || records.empty())
  {
    theCameraImage.setResolution(0, 0);
    return;
  }
  if(index >= records.size())
    index = loop ? 0 : records.size() - 1;

  const Record& record = records[index];
  if(loadedIndex != index)
  {
    if(!loadImage(record))
    {
      failed = true;
      return;
    }
    loadedIndex = index;
  }

  // Zero copy: the representation references the provider's buffer, exactly like
  // LogDataProvider does for replayed images.  width is the number of YUYV pairs.
  theCameraImage.setReference(static_cast<unsigned>(record.widthPairs), static_cast<unsigned>(record.height),
                              buffer.data(), record.timestamp);
  if(!provided)
  {
    provided = true;
    ++providedFrames;
  }
}

void MujocoReplayProvider::update(CameraInfo& theCameraInfo)
{
  if(records.empty())
    return;
  const Record& record = records[std::min(index, records.size() - 1)];
  theCameraInfo.width = record.widthPairs * 2;   // CameraInfo counts real pixels
  theCameraInfo.height = record.height;
  theCameraInfo.opticalCenter = Vector2f(record.cx, record.cy);
  theCameraInfo.openingAngleWidth = 2.f * std::atan(theCameraInfo.width / (record.fx * 2.f));
  theCameraInfo.openingAngleHeight = 2.f * std::atan(theCameraInfo.height / (record.fy * 2.f));
  theCameraInfo.updateFocalLength();
}

void MujocoReplayProvider::update(CameraMatrix& theCameraMatrix)
{
  if(records.empty())
    return;
  const Record& record = records[std::min(index, records.size() - 1)];
  Matrix3f rotation;
  for(int r = 0; r < 3; ++r)
    for(int c = 0; c < 3; ++c)
      rotation(r, c) = record.rotation[r][c];
  theCameraMatrix.rotation = RotationMatrix(rotation);
  theCameraMatrix.translation = Vector3f(record.translation[0], record.translation[1], record.translation[2]);
  theCameraMatrix.isValid = true;
  theCameraMatrix.onRead(); // refresh the cached inverse
}

void MujocoReplayProvider::update(FrameInfo& theFrameInfo)
{
  theFrameInfo.time = records.empty() ? Time::getCurrentSystemTime()
                      : records[std::min(index, records.size() - 1)].timestamp;
}

bool MujocoReplayProvider::hasMoreFrames()
{
  const MujocoReplayProvider* self = instance();
  return self && !self->failed && self->index + 1 < self->records.size();
}

size_t MujocoReplayProvider::currentFrame()
{
  const MujocoReplayProvider* self = instance();
  return self ? self->index : 0;
}

size_t MujocoReplayProvider::totalFrames()
{
  const MujocoReplayProvider* self = instance();
  return self ? self->records.size() : 0;
}

unsigned MujocoReplayProvider::framesProvided()
{
  const MujocoReplayProvider* self = instance();
  return self ? self->providedFrames.load(std::memory_order_acquire) : 0;
}

bool MujocoReplayProvider::finished()
{
  const MujocoReplayProvider* self = instance();
  if(!self || self->failed)
    return true;
  if(self->loop)
    return false;
  return self->completedFrames.load(std::memory_order_acquire) >= self->records.size();
}

bool MujocoReplayProvider::hasFailed()
{
  const MujocoReplayProvider* self = instance();
  return self && self->failed;
}

bool MujocoReplayProvider::isFrameDataComplete()
{
  const MujocoReplayProvider* self = instance();
  if(!self || self->failed)
    return false;
  return !self->provided && self->index < self->records.size();
}

void MujocoReplayProvider::waitForFrameData()
{
  MujocoReplayProvider* self = instance();
  if(!self || self->failed)
    return;
  if(self->provided)
    self->completedFrames.fetch_add(1, std::memory_order_release);
  if(self->frameDelay > 0)
    Thread::sleep(static_cast<unsigned>(self->frameDelay));

  // The frame that was just processed is done; move on to the next one.
  self->provided = false;
  if(self->index + 1 < self->records.size())
    ++self->index;
  else if(self->loop)
    self->index = 0;
}

void MujocoReplayProvider::update(BallSpecification& specification)
{
  specification.radius = records[std::min(index, records.size() - 1)].ballRadius;
}

void MujocoReplayProvider::update(RobotDimensions& dimensions)
{
  dimensions.robotDepth = 100.f; // Only used for robot output, not ball perception.
}

void MujocoReplayProvider::update(BodyContour& contour)
{
  contour.lines.clear(); // T2 self silhouette unavailable: explicit replay approximation.
}

void MujocoReplayProvider::update(FieldBoundary& boundary)
{
  boundary.isValid = false; // No native field-boundary extraction in this ball-only graph.
}

void MujocoReplayProvider::update(JerseyClassifier& classifier)
{
  classifier.detectJersey = [](RobotPercept::Robot&) {}; // Ball-only replay.
}

void MujocoReplayProvider::update(MeasurementCovariance& measurement)
{
  measurement.transformPointWithCov = [this](const Vector2f& pixel, float z, Vector2f& position, Matrix2f& covariance)
  {
    CameraInfo info;
    CameraMatrix matrix;
    update(info);
    update(matrix);
    covariance = Matrix2f::Identity(); // Deterministic 1 mm^2 replay covariance, not calibrated sensor noise.
    return Transformation::imageToRobotHorizontalPlane(pixel, z, matrix, info, position);
  };
}
