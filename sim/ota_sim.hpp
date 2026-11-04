// OTA power-loss experiments on a simulated flash. Drives the real
// bootloader and OtaManager code; only the "device reboots" glue is here.
#pragma once

#include <cstdint>
#include <random>

#include "device_sim.hpp"
#include "sim_hal.hpp"

namespace shf::sim {

enum class CutRegion : uint8_t { None, SlotErase, BodyWrite, HeaderWrite, BcbErase, BcbWrite, Count };
const char* cut_region_name(CutRegion r);

struct OtaExperiment {
  // inputs
  uint32_t new_version = 0;
  size_t image_size = 0;
  bool image_fails_self_test = false;  // trial image never confirms
  // -1: cut at a uniformly random op of the clean sequence; >= 0: that op
  int64_t cut_at_op = -1;
  double extra_cut_prob = 0.0;  // chance of another cut on each recovery attempt

  // outputs
  uint64_t clean_ops = 0;
  CutRegion first_cut = CutRegion::None;
  uint32_t cuts = 0;
  uint32_t boots = 0;
  uint32_t invalid_boots = 0;   // must stay 0
  uint32_t unexpected_errors = 0;  // OTA errors not caused by power loss
  uint32_t version_after_first_cut = 0;
  bool finished_new = false;    // new image running and confirmed
  bool rolled_back = false;     // ended on the old image after a bad trial
  uint32_t final_version = 0;
};

class OtaRig {
 public:
  OtaRig(uint64_t seed, uint32_t initial_version, size_t image_size);

  // Runs one update to exp.new_version with power cuts. The rig keeps its
  // flash between experiments, so updates can be chained.
  void run(OtaExperiment& exp);

  SimHal& hal() { return hal_; }
  uint32_t running_version() const { return running_version_; }

 private:
  // Executes the update state machine until done or the step budget runs out.
  void drive(OtaExperiment& exp, const Image& img, bool count_only);
  void reboot(OtaExperiment& exp, uint32_t old_version, uint32_t new_version, bool count_only);
  CutRegion classify();

  SimHal hal_;
  std::mt19937_64 rng_;
  BootDecision boot_{};
  uint32_t running_version_;
};

}  // namespace shf::sim
