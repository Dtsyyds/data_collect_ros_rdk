#ifndef INSPECTION_SYNC__TIMESTAMPED_RING_BUFFER_HPP_
#define INSPECTION_SYNC__TIMESTAMPED_RING_BUFFER_HPP_

#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace inspection_sync
{

// A timestamp-sorted bounded buffer. max_age and max_count are enforced on every insert.
template<typename MessageT>
class TimestampedRingBuffer
{
public:
  struct Entry
  {
    int64_t stamp_ns;
    std::shared_ptr<const MessageT> message;
  };

  struct InsertResult
  {
    bool accepted{false};
    bool out_of_order{false};
    bool late{false};
  };

  struct Stats
  {
    size_t size{0};
    uint64_t received{0};
    uint64_t rejected{0};
    uint64_t out_of_order{0};
    uint64_t late{0};
    int64_t oldest_stamp_ns{0};
    int64_t newest_stamp_ns{0};
  };

  TimestampedRingBuffer(size_t max_count, int64_t max_age_ns, int64_t late_margin_ns)
  : max_count_(max_count), max_age_ns_(max_age_ns), late_margin_ns_(late_margin_ns)
  {
    if (max_count_ == 0 || max_age_ns_ <= 0 || late_margin_ns_ < 0) {
      throw std::invalid_argument("buffer bounds must be positive");
    }
  }

  InsertResult insert(int64_t stamp_ns, std::shared_ptr<const MessageT> message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++received_;
    if (stamp_ns <= 0 || !message) {
      ++rejected_;
      return {};
    }

    InsertResult result;
    result.accepted = true;
    if (latest_seen_ns_ > 0 && stamp_ns < latest_seen_ns_) {
      result.out_of_order = true;
      ++out_of_order_;
      if (latest_seen_ns_ - stamp_ns > late_margin_ns_) {
        result.late = true;
        ++late_;
      }
    }
    latest_seen_ns_ = std::max(latest_seen_ns_, stamp_ns);
    const auto position = std::upper_bound(
      entries_.begin(), entries_.end(), stamp_ns,
      [](int64_t stamp, const Entry & entry) {return stamp < entry.stamp_ns;});
    entries_.insert(position, Entry{stamp_ns, std::move(message)});
    prune_locked();
    return result;
  }

  std::optional<Entry> nearest(int64_t target_ns, int64_t tolerance_ns) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (entries_.empty()) {
      return std::nullopt;
    }
    const auto after = std::lower_bound(
      entries_.begin(), entries_.end(), target_ns,
      [](const Entry & entry, int64_t stamp) {return entry.stamp_ns < stamp;});
    const Entry * best = nullptr;
    if (after != entries_.end()) {
      best = &*after;
    }
    if (after != entries_.begin()) {
      const auto before = std::prev(after);
      if (best == nullptr || absolute_difference(before->stamp_ns, target_ns) <=
        absolute_difference(best->stamp_ns, target_ns))
      {
        best = &*before;
      }
    }
    if (best == nullptr || absolute_difference(best->stamp_ns, target_ns) >
      static_cast<uint64_t>(tolerance_ns))
    {
      return std::nullopt;
    }
    return *best;
  }

  std::vector<Entry> range(int64_t start_ns, int64_t end_ns) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Entry> result;
    if (start_ns > end_ns) {
      return result;
    }
    const auto first = std::lower_bound(
      entries_.begin(), entries_.end(), start_ns,
      [](const Entry & entry, int64_t stamp) {return entry.stamp_ns < stamp;});
    const auto last = std::upper_bound(
      entries_.begin(), entries_.end(), end_ns,
      [](int64_t stamp, const Entry & entry) {return stamp < entry.stamp_ns;});
    result.assign(first, last);
    return result;
  }

  std::optional<std::pair<Entry, Entry>> bracket(int64_t target_ns) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (entries_.empty()) {
      return std::nullopt;
    }
    const auto after = std::lower_bound(
      entries_.begin(), entries_.end(), target_ns,
      [](const Entry & entry, int64_t stamp) {return entry.stamp_ns < stamp;});
    if (after == entries_.end()) {
      return std::nullopt;
    }
    if (after->stamp_ns == target_ns) {
      return std::make_pair(*after, *after);
    }
    if (after == entries_.begin()) {
      return std::nullopt;
    }
    return std::make_pair(*std::prev(after), *after);
  }

  Stats stats() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Stats value;
    value.size = entries_.size();
    value.received = received_;
    value.rejected = rejected_;
    value.out_of_order = out_of_order_;
    value.late = late_;
    if (!entries_.empty()) {
      value.oldest_stamp_ns = entries_.front().stamp_ns;
      value.newest_stamp_ns = entries_.back().stamp_ns;
    }
    return value;
  }

private:
  static uint64_t absolute_difference(int64_t lhs, int64_t rhs)
  {
    return lhs >= rhs ? static_cast<uint64_t>(lhs - rhs) : static_cast<uint64_t>(rhs - lhs);
  }

  void prune_locked()
  {
    const int64_t cutoff_ns = latest_seen_ns_ - max_age_ns_;
    const auto first_kept = std::lower_bound(
      entries_.begin(), entries_.end(), cutoff_ns,
      [](const Entry & entry, int64_t stamp) {return entry.stamp_ns < stamp;});
    entries_.erase(entries_.begin(), first_kept);
    if (entries_.size() > max_count_) {
      entries_.erase(entries_.begin(), entries_.begin() + (entries_.size() - max_count_));
    }
  }

  const size_t max_count_;
  const int64_t max_age_ns_;
  const int64_t late_margin_ns_;
  mutable std::mutex mutex_;
  std::vector<Entry> entries_;
  int64_t latest_seen_ns_{0};
  uint64_t received_{0};
  uint64_t rejected_{0};
  uint64_t out_of_order_{0};
  uint64_t late_{0};
};

}  // namespace inspection_sync

#endif  // INSPECTION_SYNC__TIMESTAMPED_RING_BUFFER_HPP_
