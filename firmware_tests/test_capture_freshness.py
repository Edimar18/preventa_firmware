"""Run the sketch's real frame-selection function against a simulated driver."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
CAM = ROOT / 'ESP32-CAM-FINAL/PREVENTA_ESP32_CAM/PREVENTA_ESP32_CAM.ino'


class FreshCaptureTests(unittest.TestCase):
    def test_queued_inflight_missing_and_exhausted_frames(self):
        source = CAM.read_text()
        helper = re.search(r'camera_fb_t\* captureFreshFrame\(\) \{.*?\n\}', source, re.S).group()
        harness = r'''
#include <cassert>
#include <cstdint>
#include <vector>
#include <sys/time.h>
struct camera_fb_t { timeval timestamp; };
static int64_t nowUs;
static std::vector<camera_fb_t*> queued, returned;
static unsigned gets, yields;
int64_t esp_timer_get_time() { return nowUs; }
camera_fb_t* esp_camera_fb_get() {
    ++gets;
    if (queued.empty()) return nullptr;
    auto* frame = queued.front(); queued.erase(queued.begin()); return frame;
}
void esp_camera_fb_return(camera_fb_t* frame) { returned.push_back(frame); }
void delay(unsigned) { ++yields; }
camera_fb_t frameAt(int64_t us) {
    return {{static_cast<long>(us / 1000000), static_cast<long>(us % 1000000)}};
}
void reset(std::vector<camera_fb_t*> frames, int64_t request = 1000000) {
    nowUs = request; queued = frames; returned.clear(); gets = yields = 0;
}
''' + helper + r'''
int main() {
    auto old = frameAt(100000), inflight = frameAt(999999), fresh = frameAt(1000100);
    reset({&old, &fresh});
    assert(captureFreshFrame() == &fresh);
    assert(returned == std::vector<camera_fb_t*>({&old}) && gets == 2);
    reset({&old, &inflight, &fresh});
    assert(captureFreshFrame() == &fresh);
    assert(returned == std::vector<camera_fb_t*>({&old, &inflight}));
    reset({&fresh});
    assert(captureFreshFrame() == &fresh && returned.empty() && gets == 1);
    reset({});
    assert(captureFreshFrame() == nullptr && returned.empty() && gets == 1);
    reset({&old, nullptr});
    assert(captureFreshFrame() == nullptr && returned.size() == 1 && gets == 2);
    reset({&old, &old, &old, &old, &fresh});
    assert(captureFreshFrame() == nullptr && returned.size() == 4 && gets == 4);
    auto exact = frameAt(1000000);
    reset({&exact});
    assert(captureFreshFrame() == &exact && returned.empty());
    auto longUptime = frameAt(5000000000001LL);
    reset({&old, &longUptime}, 5000000000000LL);
    assert(captureFreshFrame() == &longUptime && returned.size() == 1);
}
'''
        with tempfile.TemporaryDirectory(prefix='preventa-fresh-frame-') as temp:
            cpp, binary = Path(temp) / 'test.cpp', Path(temp) / 'test'
            cpp.write_text(harness)
            subprocess.run(['g++', '-std=c++11', '-Wall', '-Wextra', '-Werror', str(cpp), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
