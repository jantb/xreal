import Darwin

/// Asks the kernel to schedule the calling thread as real time, the class
/// audio and video threads use: it is woken within a bounded time however
/// busy the other apps keep the CPU, as long as it stays within
/// `computation` seconds of work per `period`. `constraint` is how soon after
/// waking that work must be finished. Returns whether the kernel accepted it.
/// Quality of service alone still queues the thread behind other work.
@discardableResult
public func promoteCurrentThreadToRealTime(period: Double, computation: Double, constraint: Double) -> Bool {
    var timebase = mach_timebase_info_data_t()
    mach_timebase_info(&timebase)
    func ticks(_ seconds: Double) -> UInt32 {
        UInt32(min(seconds * 1e9 * Double(timebase.denom) / Double(timebase.numer), Double(UInt32.max)))
    }
    var policy = thread_time_constraint_policy_data_t(
        period: ticks(period), computation: ticks(computation), constraint: ticks(constraint), preemptible: 1)
    let count = mach_msg_type_number_t(
        MemoryLayout<thread_time_constraint_policy_data_t>.size / MemoryLayout<integer_t>.size)
    let thread = mach_thread_self()
    defer { mach_port_deallocate(mach_task_self_, thread) }
    let result = withUnsafeMutablePointer(to: &policy) { pointer in
        pointer.withMemoryRebound(to: integer_t.self, capacity: Int(count)) {
            thread_policy_set(thread, thread_policy_flavor_t(THREAD_TIME_CONSTRAINT_POLICY), $0, count)
        }
    }
    return result == KERN_SUCCESS
}

/// Whether the calling thread has a real-time policy set.
func currentThreadIsRealTime() -> Bool {
    var policy = thread_time_constraint_policy_data_t()
    var count = mach_msg_type_number_t(
        MemoryLayout<thread_time_constraint_policy_data_t>.size / MemoryLayout<integer_t>.size)
    var isDefault: boolean_t = 0
    let thread = mach_thread_self()
    defer { mach_port_deallocate(mach_task_self_, thread) }
    let result = withUnsafeMutablePointer(to: &policy) { pointer in
        pointer.withMemoryRebound(to: integer_t.self, capacity: Int(count)) {
            thread_policy_get(thread, thread_policy_flavor_t(THREAD_TIME_CONSTRAINT_POLICY), $0, &count, &isDefault)
        }
    }
    return result == KERN_SUCCESS && isDefault == 0
}

/// Blocks the calling thread until `time`, in `monotonicNow` time, to well
/// under a millisecond; returns at once if it has passed.
public func sleep(until time: Double) {
    let remaining = time - monotonicNow()
    guard remaining > 0 else { return }
    var timebase = mach_timebase_info_data_t()
    mach_timebase_info(&timebase)
    let ticks = remaining * 1e9 * Double(timebase.denom) / Double(timebase.numer)
    mach_wait_until(mach_absolute_time() + UInt64(ticks))
}
