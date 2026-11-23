/**
 * @file Main.cpp
 *
 * Headless driver for the MuJoCo replay provider.  It builds the module graph
 * from a threads configuration, replays every frame of a robocup_mujoco bundle
 * and (when the scenario provides BallPercept) writes one JSON object per frame.
 *
 * Build (inside the B-Human checkout):
 *   cd Make/Linux && ./generate && ./compile Develop MujocoReplay
 *
 * Run:
 *   MujocoReplay --manifest <bundle>/manifest.tsv \
 *                --threads Config/Scenarios/MujocoReplay/threads.cfg \
 *                --output <bundle>/ball_percepts.jsonl
 *
 * Exit codes: 0 ok, 1 usage/configuration/IO error.
 *
 * @author robocup_mujoco integration
 */

#include "Framework/Configuration.h"
#include "Framework/Debug.h"
#include "Framework/Logger.h"
#include "Framework/ModuleContainer.h"
#include "Framework/Settings.h"
#include "Modules/Infrastructure/MujocoReplayProvider/MujocoReplayProvider.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"
#include "Streaming/FunctionList.h"
#include "Streaming/InStreams.h"
#include "Streaming/Output.h"
#include "Threads/MujocoReplay.h"

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

SystemCall::Mode SystemCall::getMode()
{
  return SystemCall::logFileReplay;
}

/**
 * The tool is usually started with --threads <repo>/Config/Scenarios/<scenario>/threads.cfg.
 * File::getBHDir() cannot derive the repository from this binary's location
 * (Build/Linux/<Target>/<Config>/), so the configuration directory is taken from
 * the threads path and put in front of the search path instead.
 */
static std::string configDirFromThreadsFile(const std::string& threadsConfig)
{
  for(const char* marker : {"/Scenarios/", "\\Scenarios\\"})
  {
    const std::size_t pos = threadsConfig.rfind(marker);
    if(pos != std::string::npos)
      return threadsConfig.substr(0, pos + 1); // keep the trailing separator
  }
  return {};
}

static void prependConfigDirs(Settings& settings, const std::string& configDir, const std::string& scenario)
{
  if(configDir.empty())
    return;
  std::vector<std::string> extra =
  {
    configDir + "Scenarios/" + scenario + "/",
    configDir + "Locations/Default/",
    configDir + "Scenarios/Default/",
    configDir + "Robots/" + settings.robotName + "/",
    configDir + "Robots/Default/",
  };
  settings.searchPath.insert(settings.searchPath.begin(), extra.begin(), extra.end());
}

/** Attach the console endpoint that SimRobot normally supplies to Debug. */
class HeadlessReplayDebug : public Debug
{
  DebugReceiver<MessageQueue> consoleSink;

public:
  HeadlessReplayDebug(const Settings& settings, const Configuration& config) :
    Debug(settings, "MujocoReplay", config), consoleSink(this, "Debug", MAX_PACKAGE_RECEIVE_SIZE)
  {
    debugSender->setReceiver(consoleSink);
  }

protected:
  bool main() override
  {
    const bool wait = Debug::main();
    consoleSink.receivePacket();
    consoleSink.clear();
    return wait;
  }
};

int main(int argc, char** argv)
{
  std::string manifest;
  std::string threadsConfig;
  std::string output;
  std::string scenario = "MujocoReplay";

  for(int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    const bool hasNext = i + 1 < argc;
    if(arg == "--manifest" && hasNext)
      manifest = argv[++i];
    else if(arg == "--threads" && hasNext)
      threadsConfig = argv[++i];
    else if(arg == "--output" && hasNext)
      output = argv[++i];
    else if(arg == "--scenario" && hasNext)
      scenario = argv[++i];
    else
    {
      OUTPUT_ERROR("Usage: MujocoReplay --manifest <manifest.tsv> --threads <threads.cfg> "
                   "--output <ball_percepts.jsonl> [--scenario <name>]");
      return EXIT_FAILURE;
    }
  }
  if(manifest.empty() || threadsConfig.empty() || output.empty())
  {
    OUTPUT_ERROR("MujocoReplay needs --manifest, --threads and --output.");
    return EXIT_FAILURE;
  }

  // Picked up by the provider/execution unit before the module graph is created.
  MujocoReplayProvider::setManifest(manifest);
  MujocoReplay::setOutput(output);
  const std::string summaryPath = output + ".summary.json";
  MujocoReplay::setSummary(summaryPath);

  FunctionList::execute();

  Configuration config;
  {
    InMapFile stream(threadsConfig);
    if(!stream.exists())
    {
      OUTPUT_ERROR(threadsConfig << ": could not open the threads configuration.");
      return EXIT_FAILURE;
    }
    stream >> config;
    if(config().empty())
    {
      OUTPUT_ERROR(threadsConfig << ": could not parse the threads configuration.");
      return EXIT_FAILURE;
    }
  }

  Settings settings("MujocoReplay", 0.01f, Settings::k1, 1, Settings::blue, Settings::blue, 1,
                    "Default", scenario, 0);
  settings.updateSearchPath();
  prependConfigDirs(settings, configDirFromThreadsFile(threadsConfig), scenario);

  std::cout << "MujocoReplay: manifest=" << manifest << std::endl
            << "              scenario=" << threadsConfig << std::endl
            << "              search path:" << std::endl;
  for(const std::string& path : settings.searchPath)
    std::cout << "                " << path << std::endl;

  // Same bootstrap as Framework/Robot.cpp: Debug owns the debug queues and, in
  // its own thread, Debug::init() computes the module graph and broadcasts the
  // initial idModuleRequest to every connected container -- that message is what
  // makes ModuleGraphRunner create the modules.  The container is therefore
  // connected before the Debug thread is started.
  File::setSearchPath(settings.searchPath);
  HeadlessReplayDebug debug(settings, config);
  Logger logger(config);
  ModuleContainer container(settings, "MujocoReplay", config, 0, &logger);
  container.connectWithDebug(&debug, config()[0]);
  File::clearSearchPath();

  debug.start();
  container.start();

  // The container drives its own thread (ThreadFrame::threadMain -> init/main/
  // terminate), so the app only waits: first for the provider to load the
  // manifest, then for the replay to finish.  Bounded so a broken graph cannot
  // hang the tool forever.
  const unsigned maxWaitForLoad = 500;   // 10 s
  const unsigned maxWaitForFrames = 15000; // 300 s
  for(unsigned waited = 0; waited < maxWaitForLoad && MujocoReplayProvider::totalFrames() == 0; ++waited)
    Thread::sleep(20);
  for(unsigned waited = 0; waited < maxWaitForFrames && !MujocoReplayProvider::finished(); ++waited)
    Thread::sleep(20);

  container.announceStop();
  debug.announceStop();
  // Debug owns senders referring to the container's receivers. Stop it before
  // container.stop() destroys the receiver queues (especially on error output).
  debug.stop();
  container.stop();

  // The module thread wrote the summary; the tool only reads it (see MujocoReplay.h).
  unsigned provided = MujocoReplayProvider::framesProvided();
  unsigned total = static_cast<unsigned>(MujocoReplayProvider::totalFrames());
  std::string summaryText;
  {
    std::ifstream summaryFile(summaryPath);
    if(summaryFile.is_open())
    {
      std::ostringstream buffer;
      buffer << summaryFile.rdbuf();
      summaryText = buffer.str();
      for(const auto& key : {"\"framesProvided\":", "\"totalFrames\":"})
      {
        const std::size_t pos = summaryText.find(key);
        if(pos == std::string::npos)
          continue;
        const unsigned value = static_cast<unsigned>(std::strtoul(summaryText.c_str() + pos + std::strlen(key), nullptr, 10));
        if(std::string(key) == "\"framesProvided\":")
          provided = value;
        else
          total = value;
      }
    }
  }

  std::cout << "MujocoReplay: provided " << provided << "/" << total << " frames, wrote "
            << MujocoReplay::writtenPercepts() << " ball percepts to " << output << std::endl;
  if(!summaryText.empty())
    std::cout << "MujocoReplay: summary " << summaryText;
  return total > 0 && provided == total && MujocoReplay::writtenPercepts() == total ? EXIT_SUCCESS : EXIT_FAILURE;
}
