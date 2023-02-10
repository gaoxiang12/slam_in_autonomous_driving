#ifndef DRIVER_VELODYNE_SYNCHRONIZED_QUEUE_H
#define DRIVER_VELODYNE_SYNCHRONIZED_QUEUE_H

#include <queue>

#include <boost/thread.hpp>

#define MAX_SYNCHRONE_QUEUE_SIZE 256

namespace driver {
namespace velodyne {
template <typename T>
class SynchronizedQueue {
 public:
  SynchronizedQueue() : end_queue_(false), queue_(), mutex_(), cond_() {}

  void Enqueue(const T& data) {
    boost::unique_lock<boost::mutex> lock(mutex_);

    if (end_queue_) {
      return;
    }

    if (queue_.size() <= MAX_SYNCHRONE_QUEUE_SIZE) {
      queue_.push(data);
    }

    cond_.notify_one();
  }

  bool Dequeue(T& result) {
    boost::unique_lock<boost::mutex> lock(mutex_);

    while (queue_.empty() && !end_queue_) {
      cond_.wait(lock);
    }

    if (end_queue_) {
      return false;
    }

    result = queue_.front();
    queue_.pop();
    // std::cout << "pop queue " << queue_.size() << std::endl;

    return true;
  }

  void Stop() {
    boost::unique_lock<boost::mutex> lock(mutex_);
    end_queue_ = true;
    Clear();
    cond_.notify_one();
  }

  unsigned int size() {
    boost::unique_lock<boost::mutex> lock(mutex_);
    return static_cast<unsigned int>(queue_.size());
  }

  bool empty() const {
    boost::unique_lock<boost::mutex> lock(mutex_);
    return (queue_.empty());
  }

 private:
  void Clear() {
    while (!queue_.empty()) {
      queue_.pop();
    }
  }

 private:
  bool end_queue_;
  std::queue<T> queue_;             // Use STL queue to store data
  mutable boost::mutex mutex_;      // The mutex to synchronise on
  boost::condition_variable cond_;  // The condition to wait for
};
}  // namespace velodyne
}  // namespace driver
#endif  // DRIVER_VELODYNE_SYNCHRONIZED_QUEUE_H