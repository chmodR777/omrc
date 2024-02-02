#pragma once
#include <atomic>
namespace OMDB
{
	class Spinlock {
	public:
		Spinlock() = default;
		Spinlock(const Spinlock&) = delete;
		Spinlock& operator=(const Spinlock) = delete;
		void lock() {
			while (flag.test_and_set(std::memory_order_acquire)) {}
		}
		void unlock() {
			flag.clear(std::memory_order_release);
		}
	private:
		std::atomic_flag flag = ATOMIC_FLAG_INIT;
	};
}
