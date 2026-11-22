/**
 * @file Threads/MujocoReplay.h
 *
 * This file declares the execution unit for the MuJoCo replay thread.
 *
 * @author robocup_mujoco integration
 */

#pragma once

#include "Tools/Framework/BHExecutionUnit.h"

#include <atomic>

#include <string>

/**
 * @class MujocoReplay
 *
 * The execution unit of a thread that replays MuJoCo camera frames.  It mirrors
 * the way \c Perception frames are driven, but uses the replay provider as the
 * frame clock instead of the camera providers:
 *
 *   beforeFrame(): a new replayed frame is ready
 *   afterFrame():  advance to the next frame (and optionally pause)
 */
class MujocoReplay : public BHExecutionUnit
{
private:
  static std::string outputPath; /**< Optional BallPercept JSONL output. */
  static std::string summaryPath; /**< Replay summary written by the module thread. */
  static std::atomic<unsigned> written; /**< Number of BallPercept records written. */

public:
  /** Set the JSONL output file (empty disables writing). */
  static void setOutput(const std::string& path) {outputPath = path;}

  /**
   * File the module thread writes the replay summary to (frame count plus the
   * native camera data of the first frame).  The tool reads it after stopping
   * the thread, which is the only race-free hand-off in this framework.
   */
  static void setSummary(const std::string& path) {summaryPath = path;}

  /** Number of BallPercept records written so far (read from the main thread). */
  static unsigned writtenPercepts() {return written.load(std::memory_order_acquire);}

  bool beforeFrame() override;
  bool afterFrame() override;
};
