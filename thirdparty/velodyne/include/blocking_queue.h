#ifndef TOOL_ANNOTATION_COMMON_BLOCKING_QUEUE_H_
#define TOOL_ANNOTATION_COMMON_BLOCKING_QUEUE_H_

#include <chrono>
#include <cstddef>
#include <deque>
#include <memory>

#include "mutex.h"

namespace driver {
namespace velodyne {

// A thread-safe blocking queue that is useful for producer/consumer patterns.
// 'T' must be movable.
template <typename T>
class BlockingQueue {
 public:
  static constexpr size_t kInfiniteQueueSize = 0;

  // Constructs a blocking queue with infinite queue size.
  BlockingQueue() : BlockingQueue(kInfiniteQueueSize) {}

  BlockingQueue(const BlockingQueue&) = delete;
  BlockingQueue& operator=(const BlockingQueue&) = delete;

  // Constructs a blocking queue with a size of 'queue_size'.
  explicit BlockingQueue(const size_t queue_size) : queue_size_(queue_size) {}

  // Pushes a value onto the queue. Blocks if the queue is full.
  void Push(T t) {
    MutexLocker lock(&mutex_);
    lock.Await([this]() { return QueueNotFullCondition(); });
    deque_.push_back(std::move(t));
  }

  // Like push, but returns false if 'timeout' is reached.
  bool PushWithTimeout(T t, const std::chrono::seconds timeout) {
    MutexLocker lock(&mutex_);
    if (!lock.AwaitWithTimeout([this]() { return QueueNotFullCondition(); },
                               timeout)) {
      return false;
    }
    deque_.push_back(std::move(t));
    return true;
  }

  // Pops the next value from the queue. Blocks until a value is available.
  T Pop() {
    MutexLocker lock(&mutex_);
    lock.Await([this]() { return !QueueEmptyCondition(); });

    T t = std::move(deque_.front());
    deque_.pop_front();
    return t;
  }

  // Like Pop, but can timeout. Returns nullptr in this case.
  T PopWithTimeout(const std::chrono::seconds timeout) {
    MutexLocker lock(&mutex_);
    if (!lock.AwaitWithTimeout([this]() { return !QueueEmptyCondition(); },
                               timeout)) {
      return nullptr;
    }
    T t = std::move(deque_.front());
    deque_.pop_front();
    return t;
  }

  void Stop() {
    MutexLocker lock(&mutex_);
    deque_.clear();
  }

  // Returns the next value in the queue or nullptr if the queue is empty.
  // Maintains ownership. This assumes a member function get() that returns
  // a pointer to the given type R.
  template <typename R>
  const R* Peek() {
    MutexLocker lock(&mutex_);
    if (deque_.empty()) {
      return nullptr;
    }
    return deque_.front().get();
  }

  // Returns the number of items currently in the queue.
  size_t Size() {
    MutexLocker lock(&mutex_);
    return deque_.size();
  }

  // Blocks until the queue is empty.
  void WaitUntilEmpty() {
    MutexLocker lock(&mutex_);
    lock.Await([this]() { return QueueEmptyCondition(); });
  }

 private:
  // Returns true iff the queue is empty.
  bool QueueEmptyCondition() { return deque_.empty(); }

  // Returns true iff the queue is not full.
  bool QueueNotFullCondition() {
    return queue_size_ == kInfiniteQueueSize || deque_.size() < queue_size_;
  }

  Mutex mutex_;
  const size_t queue_size_;
  std::deque<T> deque_;
};

}  // namespace velodyne
}  // namespace driver

#endif  // CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_
