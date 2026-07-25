#include "gtest/gtest.h"

#include "motion_data_reader.hpp"
#include "input_interface/streamed_motion_merger.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

namespace {

StreamedMotionMerger::IncomingData MakeChunk(int first_frame, int frame_count = 5) {
  StreamedMotionMerger::IncomingData data;
  data.protocol_version = 1;
  data.catch_up_enabled = true;
  data.num_frames = frame_count;
  data.num_joints = 1;
  data.num_quat_bodies = 1;
  data.num_smpl_joints = 0;
  data.num_smpl_poses = 0;

  for (int offset = 0; offset < frame_count; ++offset) {
    const int frame = first_frame + offset;
    data.frame_indices.push_back(static_cast<int64_t>(frame));
    data.joint_pos.push_back({static_cast<double>(frame)});
    data.joint_vel.push_back({1.0});
    data.body_quat.push_back({std::array<double, 4>{1.0, 0.0, 0.0, 0.0}});
  }
  return data;
}

void ApplyMergeResult(
    const StreamedMotionMerger::MergeResult& result,
    int& current_playback_frame) {
  ASSERT_NE(result.motion, nullptr);
  if (result.did_catchup_reset) {
    current_playback_frame = 0;
    return;
  }

  current_playback_frame -= result.frame_offset_adjustment;
  current_playback_frame = std::clamp(
      current_playback_frame,
      0,
      std::max(0, result.motion->timesteps - 1));
  if (result.recommended_playback_frame >= 0) {
    current_playback_frame = result.recommended_playback_frame;
  }
}

TEST(StreamedMotionMergerTest, IsaacLowLatencyModeBoundsReferenceBacklog) {
  constexpr int kTargetLagFrames = 20;
  StreamedMotionMerger merger(kTargetLagFrames);
  int current_playback_frame = 0;
  bool observed_rebase = false;

  auto result = merger.MergeIncomingData(MakeChunk(0), current_playback_frame);
  ASSERT_TRUE(result.did_catchup_reset);
  ApplyMergeResult(result, current_playback_frame);

  // Model a source that advances one frame per packet while the state-driven
  // controller advances only once for every two source packets.
  for (int first_frame = 1; first_frame <= 80; ++first_frame) {
    result = merger.MergeIncomingData(MakeChunk(first_frame), current_playback_frame);
    ASSERT_FALSE(result.did_catchup_reset);
    ApplyMergeResult(result, current_playback_frame);

    if (result.did_low_latency_rebase) {
      observed_rebase = true;
      EXPECT_EQ(result.playback_lag_frames_after, kTargetLagFrames);
      EXPECT_GT(result.playback_lag_frames_before, kTargetLagFrames);
    }

    if (first_frame % 2 == 0 && current_playback_frame + 1 < result.motion->timesteps) {
      ++current_playback_frame;
    }
  }

  EXPECT_TRUE(observed_rebase);
  EXPECT_LE(result.playback_lag_frames_after, kTargetLagFrames);
}

TEST(StreamedMotionMergerTest, LegacyModeDoesNotForceLowLatencyCursorAdvance) {
  StreamedMotionMerger merger;
  int current_playback_frame = 0;

  auto result = merger.MergeIncomingData(MakeChunk(0), current_playback_frame);
  ApplyMergeResult(result, current_playback_frame);

  for (int first_frame = 1; first_frame <= 40; ++first_frame) {
    result = merger.MergeIncomingData(MakeChunk(first_frame), current_playback_frame);
    ASSERT_FALSE(result.did_catchup_reset);
    EXPECT_FALSE(result.did_low_latency_rebase);
    EXPECT_EQ(result.recommended_playback_frame, -1);
    ApplyMergeResult(result, current_playback_frame);
  }

  EXPECT_GT(result.playback_lag_frames_after, 20);
}

}  // namespace
