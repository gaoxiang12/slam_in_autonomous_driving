
#ifndef TOOL_ANNOTATION_COMMON_MUTEX_H_
#define TOOL_ANNOTATION_COMMON_MUTEX_H_

#include <chrono>
#include <condition_variable>
#include <mutex>

namespace driver {
namespace velodyne {

// Defines an annotated mutex that can only be locked through its scoped locker
// implementation.
class Mutex {
 public:
  // A RAII class that acquires a mutex in its constructor, and
  // releases it in its destructor. It also implements waiting functionality on
  // conditions that get checked whenever the mutex is released.
  class Locker {
   public:
    Locker(Mutex* mutex) : mutex_(mutex), lock_(mutex->mutex_) {}

    ~Locker() {
      lock_.unlock();
      mutex_->condition_.notify_all();
    }

    template <typename Predicate>
    void Await(Predicate predicate) {
      mutex_->condition_.wait(lock_, predicate);
    }

    template <typename Predicate>
    bool AwaitWithTimeout(Predicate predicate, std::chrono::seconds timeout) {
      return mutex_->condition_.wait_for(lock_, timeout, predicate);
    }

   private:
    Mutex* mutex_;
    std::unique_lock<std::mutex> lock_;
  };

 private:
  std::condition_variable condition_;
  std::mutex mutex_;
};

using MutexLocker = Mutex::Locker;

}  // namespace velodyne
}  // namespace driver

#endif  // TOOL_ANNOTATION_COMMON_MUTEX_H_
