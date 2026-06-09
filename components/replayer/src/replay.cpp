// === replay.cpp ======================================================================================================
//                                               Sen Infrastructure
//                   Released under the Apache License v2.0 (SPDX-License-Identifier Apache-2.0).
//                                    See the LICENSE.txt file for more information.
//                   © Airbus SAS, Airbus Helicopters, and Airbus Defence and Space SAU/GmbH/SAS.
// =====================================================================================================================

#include "replay.h"

// component
#include "replayed_object.h"
#include "replayed_object_proxy.h"
#include "util.h"

// sen
#include "sen/core/base/assert.h"
#include "sen/core/base/class_helpers.h"
#include "sen/core/obj/detail/native_object_proxy.h"
#include "sen/core/obj/native_object.h"
#include "sen/db/creation.h"
#include "sen/db/deletion.h"
#include "sen/db/event.h"
#include "sen/db/input.h"
#include "sen/db/keyframe.h"
#include "sen/db/property_change.h"
#include "sen/kernel/component_api.h"

// generated code
#include "stl/sen/components/replayer/replayer.stl.h"
#include "stl/sen/kernel/basic_types.stl.h"

// std
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

namespace sen::components::replayer
{

ReplayImpl::ReplayImpl(std::string name, std::string path, std::unique_ptr<db::Input> input, kernel::RunApi& api)
  : ReplayBase(std::move(name), input->getSummary(), std::move(path))
  , input_(std::move(input))
  , api_(api)
  , cursor_(input_->begin())
{
  setNextStatus(ReplayStatus::stopped);
  setNextPlaybackTime(input_->getSummary().firstTime);
}

ReplayImpl::~ReplayImpl() { doStop(); }

void ReplayImpl::update(kernel::RunApi& runApi)
{
  const auto now = runApi.getTime();

  // Defense in depth behind the VirtualMasterClock fix (kernel_component.cpp):
  // a paused/halted master clock can report the max-time sentinel
  // (std::numeric_limits<int64>::max() ns). Latching it into lastUpdateTime_
  // poisons the next delta; if the sentinel persists every tick the replay
  // never advances (the park). Make a sentinel tick a pure no-op — no
  // advanceCursor, no lastUpdateTime_ capture — so lastUpdateTime_ keeps its
  // last LIVE value and the first live tick advances by a normal delta.
  if (now.sinceEpoch().getNanoseconds() == std::numeric_limits<std::int64_t>::max())
  {
    return;
  }

  auto nextStatus = getNextStatus();

  if (nextStatus == ReplayStatus::playing)
  {
    if (lastUpdateTime_.sinceEpoch().get() != 0)
    {
      auto delta = now - lastUpdateTime_;
      advanceCursor(delta);
    }
  }

  // Update lastUpdateTime_ only on playing and paused mode so advanceCursor
  // gets called with an updated delta time
  if (nextStatus == ReplayStatus::playing || nextStatus == ReplayStatus::paused)
  {
    lastUpdateTime_ = now;
  }
}

void ReplayImpl::advanceCursor(Duration delta)
{
  // Safety net (defence in depth): normal pacing advances by ~one step
  // (≤ ~50 ms) per call. An out-of-range delta means a desync (e.g. a seek that
  // left a stale baseline) — walking + flushing the spanned slice in one call
  // floods CPU/RAM and freezes the host. Hold position instead; update() re-bases
  // lastUpdateTime_ to the current clock, so the next tick advances normally.
  constexpr std::int64_t kMaxAdvanceNs = 1'000'000'000; // 1 s
  const auto deltaNs = delta.getNanoseconds();
  if (deltaNs > kMaxAdvanceNs || deltaNs < -kMaxAdvanceNs)
  {
    getLogger()->warn(
      "ignoring out-of-range replay advance of {} ns (|Δ| > {} ns) at {} — "
      "clock/cursor desync; holding to avoid an archive walk/rewind",
      deltaNs, kMaxAdvanceNs, getPlaybackTime().toUtcString());
    return;
  }

  bool cursorMoved = false;
  const auto currentTime = getPlaybackTime();
  const auto nextTime = currentTime + delta;

  while (!cursor_.atEnd() && cursor_.get().time <= nextTime)
  {
    applyCursor();
    ++cursor_;
    cursorMoved = true;
  }

  setNextPlaybackTime(nextTime);

  if (cursorMoved)
  {
    // flush any changes made by the cursor movement
    flushObjectsActivity();

    // pause if reached the end
    if (cursor_.atEnd())
    {
      pauseImpl();
    }
  }
}

void ReplayImpl::flushObjectsActivity() const
{
  for (const auto& [id, objectData]: objects_)
  {
    objectData.instance->flushPendingChanges(cursor_.get().time);
  }
}

void ReplayImpl::advanceImpl(Duration time)
{
  // can only advance if paused or stopped
  if (getStatus() == ReplayStatus::playing)
  {
    return;
  }

  advanceCursor(time);
}

void ReplayImpl::playImpl()
{
  // Compare against the PENDING status, not the committed one: a pause()
  // immediately before (e.g. the pause→seek→play of a scene jump) sets
  // nextStatus=paused but leaves the committed status at `playing` until the
  // next cycle swap. Checking getStatus() here would then no-op play() and
  // leave the replay frozen. getNextStatus() reflects the pause we just queued.
  if (getNextStatus() == ReplayStatus::playing)
  {
    return;
  }

  setNextStatus(ReplayStatus::playing);
}

void ReplayImpl::pauseImpl()
{
  // do nothing if already paused (pending) — see playImpl() on why nextStatus.
  if (getNextStatus() == ReplayStatus::paused)
  {
    return;
  }

  setNextStatus(ReplayStatus::paused);
}

void ReplayImpl::stopImpl() { doStop(); }

void ReplayImpl::doStop()
{
  // do nothing if already stopped
  if (getStatus() == ReplayStatus::stopped)
  {
    return;
  }

  // remove all objects
  while (!objects_.empty())
  {
    removeObject(objects_.begin(), objects_);
  }

  // move to the start
  cursor_ = input_->begin();
  setNextPlaybackTime(input_->getSummary().firstTime);

  // we are now stopped
  setNextStatus(ReplayStatus::stopped);
}

void ReplayImpl::seekImpl(TimeStamp time)
{
  if (time < input_->getSummary().firstTime || time > input_->getSummary().lastTime)
  {
    std::string err;
    err.append("time is not in the archive time window");
    throwRuntimeError(err);
  }

  auto maybeIndex = input_->getKeyframeIndex(time);
  if (!maybeIndex.has_value())
  {
    std::string err;
    err.append("the archive ");
    err.append(ReplayBase::getArchivePath());
    err.append(" does not have a keyframe index to go to ");
    err.append(time.toUtcString());
    throwRuntimeError(err);
  }

  // Apply the keyframe at/just-before `time`, then play forward to `time`,
  // applying every entry. Walk bounded by the absolute `time` — NOT via
  // advanceCursor(diff), which bases off getPlaybackTime() (the stale committed
  // value, since setNextPlaybackTime only writes the pending buffer) while the
  // keyframe entry's cursor time reads 0, so diff would be the whole target and
  // the walk would overshoot the archive end and park the replay. `seeking_`
  // lets the keyframe apply even though the committed status may still read
  // `playing` (the caller's pause() hasn't swapped yet); the caller stays paused
  // around the seek so update() doesn't advanceCursor and clobber the pending
  // setNextPlaybackTime before it commits.
  seeking_ = true;
  cursor_ = input_->at(maybeIndex.value());
  applyCursor();
  for (++cursor_; !cursor_.atEnd() && cursor_.get().time <= time; ++cursor_)
  {
    applyCursor();
  }
  setNextPlaybackTime(time);
  // Reset the update() baseline so the first playing tick after the seek does NOT
  // advanceCursor against a stale baseline. update() advances the cursor by
  // `delta = runApi.getTime() - lastUpdateTime_`; left at its pre-seek value (or
  // captured here from a paused clock that reports a max-time sentinel) the first
  // delta is a bogus gap → advanceCursor walks+flushes a massive archive slice
  // (CPU/RAM blowup) or rewinds playbackTime (stall). Zeroing it reuses the
  // existing "first play" guard in update() (`lastUpdateTime_.sinceEpoch()==0`):
  // that tick skips advanceCursor and re-captures the baseline from a *live*
  // clock, so playback resumes one step at a time from the seeked cursor.
  lastUpdateTime_ = {};
  flushObjectsActivity();
  seeking_ = false;
}

void ReplayImpl::applyCursor()
{
  const auto& entry = cursor_.get();
  std::visit(Overloaded {[this, &entry](const auto& value) { apply(entry.time, value); }}, entry.payload);
}

void ReplayImpl::apply(TimeStamp time, std::monostate value) const
{
  std::ignore = time;
  std::ignore = value;
}

void ReplayImpl::apply(TimeStamp time, const db::End& value)
{
  std::ignore = time;
  std::ignore = value;
  pauseImpl();
}

void ReplayImpl::apply(TimeStamp time, const db::PropertyChange& value)
{
  auto itr = objects_.find(value.getObjectId());
  if (itr == objects_.end())
  {
    expectedObject(time, value.getObjectId(), {"property change ", value.getProperty()->getName()});
  }
  else
  {
    itr->second.instance->inject(time, value);
  }
}

void ReplayImpl::apply(TimeStamp time, const db::Event& value)
{
  auto itr = objects_.find(value.getObjectId());
  if (itr == objects_.end())
  {
    expectedObject(time, value.getObjectId(), {"event ", value.getEvent()->getName()});
  }
  else
  {
    itr->second.instance->inject(time, value);
  }
}

void ReplayImpl::apply(TimeStamp time, const db::Keyframe& value)
{
  // Do not apply keyframes during normal playback — but DO apply them while
  // seeking (seekImpl rebuilds state from a keyframe even though the replay
  // stays in `playing` status).
  if (getStatus() == ReplayStatus::playing && !seeking_)
  {
    return;
  }

  decltype(objects_) nextObjects;

  for (const auto& snapshot: value.getSnapshots())
  {
    auto itr = objects_.find(snapshot.getObjectId());
    if (itr == objects_.end())
    {
      addObject(time, snapshot, nextObjects);
    }
    else
    {
      itr->second.instance->inject(time, snapshot);
      nextObjects.insert(*itr);
      objects_.erase(itr);
    }
  }

  // if objects remain here, they were not in the keyframe, so they get removed
  while (!objects_.empty())
  {
    removeObject(objects_.begin(), objects_);
  }

  // use the 'new' objects
  objects_ = std::move(nextObjects);
}

void ReplayImpl::apply(TimeStamp time, const db::Creation& value)
{
  const auto& snapshot = value.getSnapshot();
  auto itr = objects_.find(snapshot.getObjectId());
  if (itr != objects_.end())
  {
    getLogger()->warn(
      "ignoring creation object with repeated id {} at time {}", snapshot.getObjectId().get(), time.toUtcString());
  }
  else
  {
    addObject(time, snapshot, objects_);
  }
}

void ReplayImpl::apply(TimeStamp time, const db::Deletion& value)
{
  auto itr = objects_.find(value.getObjectId());
  if (itr == objects_.end())
  {
    expectedObject(time, value.getObjectId(), {"deletion"});
  }
  else
  {
    removeObject(itr, objects_);
  }
}

void ReplayImpl::expectedObject(TimeStamp time, ObjectId id, const std::vector<std::string_view>& reasons) const
{
  std::string msg;
  for (const auto& elem: reasons)
  {
    msg.append(elem);
  }

  getLogger()->warn("at time {} expected object with id {}: {}", time.toUtcString(), id.get(), msg);
}

void ReplayImpl::addObject(TimeStamp time, const db::Snapshot& snapshot, ObjectMap& map)
{
  snapshot.getType()->setLocalProxyMaker(
    [](NativeObject* instance, const std::string& name) -> std::shared_ptr<impl::NativeObjectProxy>
    {
      auto* owner = dynamic_cast<ReplayedObject*>(instance);
      return std::make_shared<ReplayedObjectProxy>(owner, name);
    });

  auto instance = std::make_shared<ReplayedObject>(snapshot, time);

  // add it to the bus
  auto bus = api_.getSource(kernel::BusAddress {snapshot.getSessionName(), snapshot.getBusName()});
  bus->add(instance);

  // save the references
  map.insert({snapshot.getObjectId(), {std::move(instance), bus}});
}

void ReplayImpl::removeObject(ObjectMap::iterator itr, ObjectMap& map) const
{
  itr->second.bus->remove(itr->second.instance);
  map.erase(itr);
}

}  // namespace sen::components::replayer
