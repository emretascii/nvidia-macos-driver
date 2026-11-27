#!/usr/bin/env python3
"""A display wake whose first modeset is refused recovers automatically without leaving the panel black.

Real failure trace (RTX 5090, Samsung Odyssey G81SF 4K@240Hz HDMI 2.1 FRL + DSC, macOS 26.7.1):
  1. Display sleep calls headOff() (fHeadSlept = true, fPowerOn = false).
  2. While asleep, an HPD pulse ran hotplugFire -> getDynamicDisplayInfo -> SET_HDMI_FRL_CONFIG (0x73029a),
     which timed out (0x65) against the sleeping sink and wiped NVKMS's FRL caps.
  3. setPowerState(1) ran reflip() with 4K@240Hz DSC, NVKMS refused ioctl 9, fHeadSlept stayed true, and
     nothing scheduled a retry.
  4. Later HPD restored FRL/240Hz and WindowServer called setDisplayMode(0x10c00), but hotplugFire ignored
     fPowerOn && fHeadSlept, relight gated on !fHeadSlept, and setDisplayMode returned success on fModeSet
     without calling setHeadPower(true). Every flip was rejected until a manual mode change forced switchMode
     (which also left fHeadSlept true).

Exercises the retry policy from nvrm-fb-safety.h, compiles the production wake-retry methods lifted from
nvrm-fb.cpp, and verifies wiring with a negative control against the pre-fix source.
"""
import pathlib
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else pathlib.Path(__file__).resolve().parents[1]
REL = "kexts/NVRMFB/fb/nvrm-fb.cpp"
fails = []


def arm(name, ok):
    print(("ok   " if ok else "FAIL ") + name)
    if not ok:
        fails.append(name)


def extract(text, marker):
    start = text.index(marker)
    opening = text.index("{", start)
    depth = 0
    for pos in range(opening, len(text)):
        if text[pos] == "{":
            depth += 1
        elif text[pos] == "}":
            depth -= 1
            if not depth:
                return text[start:pos + 1]
    raise AssertionError(f"unclosed production function: {marker}")


def recovery_arms(src):
    hp = extract(src, "void NVRMNVDAFramebuffer::hotplugFire(")
    sdm = extract(src, "IOReturn NVRMNVDAFramebuffer::setDisplayMode(")
    sm = extract(src, "IOReturn NVRMNVDAFramebuffer::switchMode(")
    ss = extract(src, "IOReturn NVRMNVDAFramebuffer::switchSurround(")
    am = extract(src, "bool NVRMNVDAFramebuffer::applyMode(")
    sps = extract(src, "IOReturn NVRMNVDAFramebuffer::setPowerState(")
    gwf = extract(src, "void NVRMNVDAFramebuffer::gpuWakeFire(")
    st = extract(src, "bool NVRMNVDAFramebuffer::start(")
    sp = extract(src, "void NVRMNVDAFramebuffer::stop(")
    return {
        "setPowerState(1) schedules wake retries when setHeadPower(true) fails":
            "if (!setHeadPower(true)) scheduleWakeRetry();" in sps,
        "setPowerState(1) replays a hot-plug probe deferred during display sleep":
            "if (fHotplugDeferred) { fHotplugDeferred = false; hotplugKick(); }" in sps,
        "gpuWakeFire schedules wake retries when setHeadPower(true) fails":
            "if (!ok) me->scheduleWakeRetry();" in gwf,
        "hotplugFire defers probing a known display while asleep":
            "!fb->fPowerOn && fb->fHeadSlept" in hp and "fb->fHotplugDeferred = true;" in hp
            and hp.index("fb->fHotplugDeferred = true;") < hp.index("nvrmDynamicDisplayInfo("),
        "hotplugFire skips display-audio republishing on sleeping displays":
            "if (gFBs[i] && !gFBs[i]->fDpyLost && gFBs[i]->fPowerOn) gFBs[i]->publishDisplayAudioProperties();" in hp,
        "hotplugFire relights reconnected/swapped displays using fPowerOn and setHeadPower when fHeadSlept":
            "fb->fPowerOn" in hp and "fb->fHeadSlept ? fb->setHeadPower(true) : fb->reflip()" in hp,
        "hotplugFire relights connected awake displays whose head is still slept":
            "fb->fPowerOn && fb->fHeadSlept && fb->fModeSet && fb->fSurf" in hp and "rewake[" in hp,
        "setDisplayMode relights an unlit head on same-mode calls":
            sdm.count("if (fHeadSlept && fPowerOn) (void)setHeadPower(true);") == 2,
        "switchMode clears fHeadSlept after committing a live head":
            "fHeadSlept = false;" in sm,
        "switchSurround clears fHeadSlept after committing live heads":
            "fHeadSlept = false;" in ss,
        "applyMode clears fHeadSlept after committing a live head":
            "fHeadSlept = false;" in am,
        "fWakeRetry is allocated in start() and cancelled/drained in stop()":
            "fWakeRetry = nvrmfbCall(wakeRetryFire, this);" in st and "&fWakeRetry" in sp,
    }


src = (ROOT / REL).read_text(encoding="utf-8")
for name, ok in recovery_arms(src).items():
    arm(name, ok)

# Compile and run the retry schedule + extracted production wake-retry methods.
set_head = extract(src, "bool NVRMNVDAFramebuffer::setHeadPower(")
set_pwr = extract(src, "IOReturn NVRMNVDAFramebuffer::setPowerState(")
gpu_wake = extract(src, "void NVRMNVDAFramebuffer::gpuWakeFire(")
sched_retry = extract(src, "void NVRMNVDAFramebuffer::scheduleWakeRetry(")
retry_fire = extract(src, "void NVRMNVDAFramebuffer::wakeRetryFire(")

fixture = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include "nvrm-fb-safety.h"

using IOReturn = int;
using NvBool = unsigned char;
using NvU32 = unsigned;
using thread_call_t = void *;
using thread_call_param_t = void *;
struct IOService {};
struct NvKmsKapiFunctionsTable {};
struct NvKmsKapiDevice {};
struct NvKmsKapiDynamicDisplayParams { NvU32 handle; NvBool connected; };
enum { kIOPMAckImplied = 0, kIOFBNotifyDidPowerOn = 1, kIOFBNotifyWillPowerOff = 2 };
enum { kMillisecondScale = 1 };

static bool gLockAvail = true;
static bool gDynOk = true;
static bool gDynConnected = true;
static unsigned gDynQueries = 0;
static std::recursive_mutex gDisplayLock;

struct NVRMDisplayGate {
    NVRMDisplayGate() { gDisplayLock.lock(); }
    ~NVRMDisplayGate() { gDisplayLock.unlock(); }
    bool valid() const { return gLockAvail; }
};

struct IORecursiveLock { std::recursive_mutex m; };
static IORecursiveLock gStartLock;
static IORecursiveLock *startLock() { return &gStartLock; }
static void IORecursiveLockLock(IORecursiveLock *l) { l->m.lock(); }
static void IORecursiveLockUnlock(IORecursiveLock *l) { l->m.unlock(); }

static void *IOMalloc(size_t n) { return std::malloc(n); }
static void IOFree(void *p, size_t) { std::free(p); }
static void nvu_zero(void *p, size_t n) { std::memset(p, 0, n); }
static bool PE_parse_boot_argn(const char *, char *, unsigned long) { return false; }
static int OSAddAtomic(int n, int *v) { int o = *v; *v += n; return o; }
static void clock_interval_to_deadline(unsigned ms, int, uint64_t *d) { *d = ms; }
static NvBool nvrmDynamicDisplayInfo(NvKmsKapiFunctionsTable *, NvKmsKapiDevice *, NvKmsKapiDynamicDisplayParams *dd) {
    gDynQueries++;
    if (!gDynOk) return 0;
    dd->connected = gDynConnected ? 1 : 0;
    return 1;
}
#define FBLOG(...) ((void)0)
#define FBLOGV(...) ((void)0)

struct NVRMNVDAFramebuffer {
    int refs = 1;
    void retain() { ++refs; }
    void release() { --refs; }

    NvKmsKapiFunctionsTable kmsStub{};
    NvKmsKapiDevice devStub{};
    NvKmsKapiFunctionsTable *fKms = &kmsStub;
    NvKmsKapiDevice *fDev = &devStub;
    NvU32 fDisplay = 0x100;
    void *fSurf = this;
    unsigned fHead = 1;
    bool fHeadSlept = false, fModeSet = true, fPowerOn = true, fSysSlept = false, pagingState = true;
    bool fHotplugDeferred = false;
    bool sleepOK = true, wakeOK = true, gpuReady = true;
    int fStopping = 0, fReflipRc = 0;
    thread_call_t fWakeRetry = reinterpret_cast<void *>(1);
    unsigned fWakeRetries = 0;

    unsigned sleeps = 0, wakes = 0, audioPublishes = 0, notifications = 0, familyWakes = 0;
    unsigned scheduledCount = 0;
    uint64_t lastDeadlineMs = 0;
    static inline unsigned hotplugKicks = 0;

    void scheduleCall(thread_call_t, uint64_t deadline) { scheduledCount++; lastDeadlineMs = deadline; retain(); }
    void publishDisplayAudioProperties() { audioPublishes++; }
    void ddcSetPowered(bool) {}
    bool headOff() { sleeps++; return sleepOK; }
    bool reflip() { wakes++; fReflipRc = wakeOK ? 0 : -1; return wakeOK; }
    void handleEvent(int ev) { if (ev == kIOFBNotifyDidPowerOn) notifications++; }
    bool rmGpuReady() { return gpuReady; }
    void familyDidWake() { familyWakes++; notifications++; pagingState = true; }
    static void hotplugKick() { hotplugKicks++; }

    bool setHeadPower(bool awake);
    IOReturn setPowerState(unsigned long ordinal, IOService *device);
    static void gpuWakeFire(thread_call_param_t p0, thread_call_param_t);
    void scheduleWakeRetry();
    static void wakeRetryFire(thread_call_param_t p0, thread_call_param_t);
};

class NVRMCallReference {
    NVRMNVDAFramebuffer *p;
public:
    explicit NVRMCallReference(NVRMNVDAFramebuffer *v) : p(v) {}
    ~NVRMCallReference() { p->release(); }
};
'''

checks = r'''
int main() {
    // 1. Bounded retry ramp from nvrm-fb-safety.h
    unsigned totalMs = 0, count = 0;
    for (unsigned a = 0; ; a++) {
        unsigned ms = nvrmWakeRetryDelayMs(a);
        if (!ms) break;
        totalMs += ms;
        count++;
    }
    assert(count == kNvrmWakeRetries && count == 9 && totalMs == 20000);
    assert(nvrmWakeRetryDelayMs(kNvrmWakeRetries) == 0 && nvrmWakeRetryDelayMs(100) == 0);

    // 2. Display sleep -> failed immediate wake -> disconnected retry skips reflip -> connected retry succeeds
    NVRMNVDAFramebuffer f;
    f.setPowerState(0, nullptr);
    assert(f.fHeadSlept && !f.fPowerOn);

    f.wakeOK = false;
    f.setPowerState(1, nullptr);
    assert(f.fHeadSlept && f.fPowerOn && f.wakes == 1 && f.scheduledCount == 1 && f.lastDeadlineMs == 500 && f.refs == 2);

    // Retry 1 while monitor still disconnected: queries dynamic info, skips doomed reflip, schedules retry 2
    gDynConnected = false;
    unsigned wakesBefore = f.wakes, dynBefore = gDynQueries;
    NVRMNVDAFramebuffer::wakeRetryFire(&f, nullptr);
    assert(gDynQueries == dynBefore + 1 && f.wakes == wakesBefore && f.fHeadSlept);
    assert(f.scheduledCount == 2 && f.lastDeadlineMs == 1000 && f.refs == 2);

    // Retry 2 while monitor connected and FRL/DSC ready: lights head, clears fHeadSlept, resets fWakeRetries
    gDynConnected = true;
    f.wakeOK = true;
    NVRMNVDAFramebuffer::wakeRetryFire(&f, nullptr);
    assert(!f.fHeadSlept && f.fWakeRetries == 0 && f.wakes == wakesBefore + 1 && f.audioPublishes == 1);
    assert(f.scheduledCount == 2 && f.refs == 1);

    // 3. Exhausting all retries stops cleanly (never spins forever when a monitor is switched off)
    f.setPowerState(0, nullptr);
    f.wakeOK = false;
    f.setPowerState(1, nullptr);
    assert(f.scheduledCount == 3 && f.fWakeRetries == 1 && f.refs == 2);
    for (unsigned step = 1; step < kNvrmWakeRetries; step++) {
        unsigned sc = f.scheduledCount;
        NVRMNVDAFramebuffer::wakeRetryFire(&f, nullptr);
        assert(f.scheduledCount == sc + 1 && f.fWakeRetries == step + 1 && f.refs == 2);
    }
    unsigned scFinal = f.scheduledCount;
    NVRMNVDAFramebuffer::wakeRetryFire(&f, nullptr);
    assert(f.scheduledCount == scFinal && f.fHeadSlept && f.refs == 1);

    // 4. Going back to sleep cancels pending wake retry cleanly without lighting the panel
    f.setPowerState(1, nullptr);
    assert(f.fHeadSlept && f.refs == 2);
    f.setPowerState(0, nullptr);
    f.wakeOK = true;
    wakesBefore = f.wakes;
    NVRMNVDAFramebuffer::wakeRetryFire(&f, nullptr);
    assert(f.fHeadSlept && !f.fPowerOn && f.wakes == wakesBefore && f.refs == 1);

    // 5. System resume gpuWakeFire also schedules wake retries if the initial reflip fails
    f.fPowerOn = true;
    f.wakeOK = false;
    unsigned scBefore = f.scheduledCount;
    f.retain();
    NVRMNVDAFramebuffer::gpuWakeFire(&f, nullptr);
    assert(f.fHeadSlept && f.scheduledCount == scBefore + 1 && f.refs == 2);
    f.wakeOK = true;
    NVRMNVDAFramebuffer::wakeRetryFire(&f, nullptr);
    assert(!f.fHeadSlept && f.fWakeRetries == 0 && f.refs == 1);

    puts("PASS: wake retry ramp, dynamic-info refresh, disconnect skip, recovery, bounded stop, sleep cancellation, gpuWakeFire retry");
}
'''

with tempfile.TemporaryDirectory() as td:
    td = pathlib.Path(td)
    cpp = td / "wake_recovery.cpp"
    cpp.write_text(
        fixture + "\n" + set_head + "\n" + set_pwr + "\n" + gpu_wake + "\n" + sched_retry + "\n" + retry_fire + "\n" + checks,
        encoding="utf-8",
    )
    r = subprocess.run(
        ["clang++", "-std=c++17", "-Wall", "-Wextra", "-Wno-unused-parameter", "-Werror", "-pthread", f"-I{ROOT}/kexts/NVRMFB/fb", str(cpp), "-o", str(td / "t")],
        capture_output=True,
        text=True,
    )
    if r.returncode:
        print(r.stderr[-2000:])
        arm("compiled wake-recovery behavioral harness builds cleanly", False)
    else:
        run = subprocess.run([str(td / "t")], capture_output=True, text=True)
        if run.stdout:
            print(run.stdout.strip())
        if run.returncode:
            print(run.stderr[-2000:])
        arm("compiled wake-recovery behavioral harness passes", run.returncode == 0)


def old_source():
    dirty = subprocess.run(["git", "-C", str(ROOT), "diff", "--quiet", "HEAD", "--", REL]).returncode != 0
    if dirty:
        rev = "HEAD"
    else:
        log = subprocess.run(
            ["git", "-C", str(ROOT), "log", "--format=%H", "-S", "fHotplugDeferred", "--", REL],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.split()
        rev = log[-1] + "~1"
    return subprocess.run(
        ["git", "-C", str(ROOT), "show", f"{rev}:{REL}"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout


try:
    old = recovery_arms(old_source())
    arm("negative control: every wake-recovery arm fails on the pre-fix source", not any(old.values()))
except (subprocess.CalledProcessError, ValueError, IndexError) as e:
    arm(f"negative control could not run ({e.__class__.__name__})", False)

print(f"{len(fails)} failed")
sys.exit(1 if fails else 0)
