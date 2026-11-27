"""Exercise the actual UART receiver, CAM dispatcher and preview sender on host."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / 'ESP32-MAIN-FINAL/PREVENTA_ESP32_Main'
CAM = ROOT / 'ESP32-CAM-FINAL/PREVENTA_ESP32_CAM'


def function(source, signature):
    start = source.index(signature + ' {')
    return source[start:source.index('\n}', start) + 2]


COMMON = r'''
#include <cassert>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>
#include "camera_transport.h"
static uint32_t clockMs = 1000;
uint32_t millis() { return clockMs; }
void delay(unsigned ms) { clockMs += ms; }
size_t strlcpy(char* dest, const char* src, size_t size) {
    if (size) { std::strncpy(dest,src,size-1); dest[size-1]=0; }
    return std::strlen(src);
}
class String {
    std::string value;
public:
    String(const char* text="") : value(text) {}
    String(std::string text) : value(text) {}
    const char* c_str() const { return value.c_str(); }
    size_t length() const { return value.size(); }
    void trim() {
        const auto first=value.find_first_not_of(" \t\r\n");
        if(first==std::string::npos){value.clear();return;}
        value=value.substr(first,value.find_last_not_of(" \t\r\n")-first+1);
    }
    bool startsWith(const char* prefix) const { return value.rfind(prefix,0)==0; }
    bool endsWith(const char* suffix) const {
        const size_t length=std::strlen(suffix);
        return value.size()>=length && value.compare(value.size()-length,length,suffix)==0;
    }
    String substring(size_t start) const { return String(value.substr(start)); }
    int indexOf(char c) const { auto p=value.find(c); return p==std::string::npos?-1:int(p); }
    String& operator+=(char c) { value+=c; return *this; }
    friend String operator+(const String& a,const String& b) { return String(a.value+b.value); }
    bool operator==(const char* other) const { return value==other; }
};
struct SerialMock {
    std::deque<uint8_t> rx;
    std::string tx;
    size_t available() const { return rx.size(); }
    int read() { int c=rx.front(); rx.pop_front(); return c; }
    void feed(const std::string& text) { for(auto c:text)rx.push_back(uint8_t(c)); }
    void feed(const std::vector<uint8_t>& bytes) { for(auto c:bytes)rx.push_back(c); }
    void println(const char* text) { tx+=text;tx+='\n'; }
    void printf(const char* format,...) {
        char buffer[1024];va_list args;va_start(args,format);vsnprintf(buffer,sizeof(buffer),format,args);va_end(args);tx+=buffer;
    }
    size_t write(const uint8_t* bytes,size_t n) { tx.append(reinterpret_cast<const char*>(bytes),n);return n; }
    void flush() {}
};
'''


class CameraRecoveryTests(unittest.TestCase):
    def compile_run(self, source, includes):
        with tempfile.TemporaryDirectory(prefix='preventa-camera-recovery-') as temp:
            cpp, binary = Path(temp)/'test.cpp', Path(temp)/'test'
            cpp.write_text(source)
            subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror',
                            '-Wno-unused-function','-I',str(includes),str(cpp),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)

    def test_actual_main_receiver_recovers_from_incomplete_corrupted_and_late_frames(self):
        source=(MAIN/'PREVENTA_ESP32_Main.ino').read_text()
        receiver=source[source.index('void clearSetupPreview() {'):]
        receiver=receiver[:receiver.index('\n// ============================================================================')]
        harness=COMMON+r'''
SerialMock Serial,camSerial;
struct { bool cameraEnabled=true; } g_remote_settings;
enum class SetupResult { Idle,Pending,Accepted,Failed };
bool g_setup_capture_pending=false,g_cam_calibration_pending=false,g_cam_query_pending=false;
bool g_cam_image_streaming=false,g_cam_reset_receiver=false,g_portal_active=true,g_cam_present=true,g_cam_sensor_fault=false;
uint32_t g_cam_rx_bytes=0;
bool g_cam_ping_pending=false;
uint32_t g_cam_ping_revision=0,g_cam_ping_started_ms=0,g_cam_last_ping_ms=0;
uint32_t g_cam_request_id=0,g_cam_active_preview_id=0,g_cam_reply_revision=0;
uint32_t g_cam_query_sent_ms=0,g_last_cam_query_ms=0,g_setup_photo_revision=0;
int g_cam_latest_fire_pct=0,g_cam_latest_state=0;
SetupResult g_setup_capture_result=SetupResult::Idle,g_setup_cam_calibration_result=SetupResult::Idle;
char g_setup_capture_message[144]={},g_setup_cam_calibration_message[144]={};
uint8_t *g_setup_photo=nullptr,*g_setup_model_input=nullptr;
size_t g_setup_photo_size=0;
constexpr int kModelInputSize=96;
constexpr size_t kModelInputBmpBytes=54+96*96*3;
constexpr uint32_t CAM_IMAGE_STALL_TIMEOUT_MS=10000;
struct { int fire_pct=-1,no_fire_pct=-1; uint32_t response_ms=0; } g_setup_inference;
uint32_t esp_random(){ static uint32_t next=70;return ++next; }
void flushDeferredReadingToBackend() {}
void applyCameraSetting() {}
int uploads=0;
bool uploadCaptureToBackend(const uint8_t*,size_t,float){++uploads;return true;}
'''+receiver+function(source,'void pollCameraLink()')+r'''
void incoming(const std::string& text) { camSerial.feed(text);processCamSerial(); }
void incoming(const std::vector<uint8_t>& bytes) { camSerial.feed(bytes);processCamSerial(); }
std::vector<uint8_t> bmp(){std::vector<uint8_t> data(kModelInputBmpBytes,40);data[0]='B';data[1]='M';data[18]=96;data[22]=96;return data;}
void start() {
    if(g_cam_query_pending)failCameraRequest("test reset");
    clearSetupPreview();camSerial.rx.clear();clockMs+=100;
    g_setup_cam_calibration_result=SetupResult::Idle;
    assert(requestCameraCapture(true));processCamSerial();
}
void begin(uint32_t id,int fire=75){incoming("CAPTURE_BEGIN:"+std::to_string(id)+"\nPREDICT:"+std::to_string(fire)+","+std::to_string(100-fire)+"\n");}
void payload(const std::vector<uint8_t>& data){incoming("MODEL_INPUT:"+std::to_string(data.size())+"\n");incoming(data);}
void end(uint32_t id,uint32_t checksum){incoming("\nCAPTURE_END:"+std::to_string(id)+","+std::to_string(checksum)+"\n");}
int main(){
    assert(preventa::cameraPreviewChecksum(reinterpret_cast<const uint8_t*>("hello"),5)==0x4f9f2cab);
    const uint32_t nearWrap=UINT32_MAX-1000;
    assert(!preventa::cameraDeadlineExpired(nearWrap+44999U,nearWrap));
    assert(preventa::cameraDeadlineExpired(nearWrap+45000U,nearWrap));
    auto image=bmp();
    start();auto id=g_cam_request_id;
    assert(camSerial.tx.find("PREVIEW:"+std::to_string(id))!=std::string::npos);
    begin(id);payload(image);
    assert(g_setup_capture_result==SetupResult::Pending);
    incoming("PONG\nSTANDBY_OK\n");assert(g_cam_query_pending);
    end(id,preventa::cameraPreviewChecksum(image.data(),image.size()));
    assert(g_setup_capture_result==SetupResult::Accepted && g_setup_photo_revision==1);
    assert(g_setup_model_input && g_setup_inference.fire_pct==75 && !g_cam_query_pending);

    start();id=g_cam_request_id;
    begin(id-1,5);payload(image);end(id-1,preventa::cameraPreviewChecksum(image.data(),image.size()));
    assert(g_setup_capture_pending && !g_setup_model_input && g_setup_inference.fire_pct==-1);
    begin(id,80);payload(image);end(id,preventa::cameraPreviewChecksum(image.data(),image.size()));
    assert(g_setup_capture_result==SetupResult::Accepted && g_setup_inference.fire_pct==80);

    start();id=g_cam_request_id;begin(id);payload(image);end(id,0);
    assert(g_setup_capture_result==SetupResult::Failed && !g_setup_model_input);

    start();id=g_cam_request_id;begin(id);
    incoming("MODEL_INPUT:27702\n");incoming(std::vector<uint8_t>(128,4));
    clockMs+=10001;processCamSerial();
    assert(g_setup_capture_result==SetupResult::Failed && !g_cam_image_streaming && !g_cam_query_pending);

    start();clockMs+=45000;camSerial.feed(std::string(5000,'X'));processCamSerial();
    assert(g_setup_capture_result==SetupResult::Failed && !g_cam_query_pending);
    incoming("\n");start();id=g_cam_request_id;begin(id);payload(image);end(id,preventa::cameraPreviewChecksum(image.data(),image.size()));
    assert(g_setup_capture_result==SetupResult::Accepted);

    start();id=g_cam_request_id;begin(id);
    assert(requestCameraRecalibration());const auto resetId=g_cam_request_id;processCamSerial();
    assert(g_cam_calibration_pending && g_setup_cam_calibration_result==SetupResult::Pending);
    end(id,1);incoming("CAM_RECALIBRATED:"+std::to_string(id)+",OK\n");
    assert(g_cam_calibration_pending);
    incoming("CAM_RECALIBRATED:"+std::to_string(resetId)+",OK\n");
    assert(!g_cam_calibration_pending && !g_cam_query_pending && g_cam_present);
    assert(g_setup_cam_calibration_result==SetupResult::Accepted);

    start();incoming("CAM_READY:1\n");
    assert(g_setup_capture_result==SetupResult::Failed);
    start();id=g_cam_request_id;
    incoming("CAPTURE_ERROR:"+std::to_string(id)+",preview_memory\n");
    assert(g_setup_capture_result==SetupResult::Failed && std::strstr(g_setup_capture_message,"preview_memory"));
    clearSetupPreview();g_cam_reset_receiver=true;processCamSerial();
    clockMs+=10001;pollCameraLink();assert(g_cam_ping_pending);
    incoming("PONG\nSTATUS:ERR,PSRAM:1000000,STBY:0\n");pollCameraLink();
    assert(!g_cam_ping_pending && !g_cam_present && g_cam_sensor_fault);
    incoming("PONG\n");assert(!g_cam_present);
    incoming("STATUS:OK,PSRAM:1000000,STBY:0\n");assert(g_cam_present && !g_cam_sensor_fault);
    clockMs+=10001;pollCameraLink();assert(g_cam_ping_pending);
    clockMs+=1501;pollCameraLink();assert(!g_cam_ping_pending && !g_cam_present);
    assert(g_cam_rx_bytes>27702);
    g_remote_settings.cameraEnabled=false;assert(!requestCameraCapture(true));assert(!requestCameraRecalibration());
}
'''
        self.compile_run(harness,MAIN)

    def test_actual_cam_preview_and_reset_dispatch_preserve_model_and_omit_full_jpeg(self):
        source=(CAM/'PREVENTA_ESP32_CAM.ino').read_text()
        signatures=['void captureSetupPreview(uint32_t requestId)', 'void recalibrateCameraSensor(uint32_t requestId)',
                    'bool cameraCommandId(const char* text, uint32_t& value)', 'void handleCommand(const char* cmd)']
        harness=COMMON+r'''
SerialMock Serial;
constexpr size_t kModelInputBmpBytes=54+96*96*3;
bool g_cam_initialized=true,g_inference_running=false,g_standby_mode=false;
uint32_t g_last_auto_capture_ms=0;
int model=1,inputTensor=2,outputTensor=3;
int *interpreter=&model,*input=&inputTensor,*output=&outputTensor;
struct camera_fb_t { int value; } frame;
int returned=0,restarted=0,settled=0,legacyCalls=0;
bool captureOk=true,memoryOk=true,inferenceOk=true,sensorOk=true;
void flashPulseOn(){} void flashPulseOff(){}
camera_fb_t* captureFreshFrame(){return captureOk?&frame:nullptr;}
void esp_camera_fb_return(camera_fb_t*){++returned;}
void* ps_malloc(size_t n){return memoryOk?malloc(n):nullptr;}
bool runInference(camera_fb_t*,float& fire,float& noFire,uint8_t* preview){
    if(!inferenceOk)return false;
    memset(preview,35,kModelInputBmpBytes);preview[0]='B';preview[1]='M';
    fire=.75f;noFire=.25f;return true;
}
bool initializeCameraSensor(){++restarted;return sensorOk;}
bool settleCameraSensor(){++settled;return sensorOk;}
void captureAndSend(bool,bool){++legacyCalls;}
struct { unsigned getFreePsram(){return 1000000;} } ESP;
'''+ '\n'.join(function(source,sig) for sig in signatures)+r'''
int main(){
    handleCommand("PREVIEW:1234");
    assert(Serial.tx.find("CAPTURE_BEGIN:1234\n")!=std::string::npos);
    assert(Serial.tx.find("PREDICT:75,25\nMODEL_INPUT:27702\n")!=std::string::npos);
    assert(Serial.tx.find("CAPTURE_END:1234,")!=std::string::npos);
    assert(Serial.tx.find("IMG:")==std::string::npos && returned==1 && !g_inference_running);
    const auto before=Serial.tx.size();handleCommand("PREVIEW:0");handleCommand("PREVIEW:123junk");assert(Serial.tx.size()==before);
    handleCommand("PREDICT");assert(legacyCalls==1);
    handleCommand("CAM_RECALIBRATE:4455");
    assert(restarted==1 && settled==1 && g_cam_initialized && !g_inference_running);
    assert(interpreter==&model && input==&inputTensor && output==&outputTensor);
    assert(Serial.tx.find("CAM_RECALIBRATED:4455,OK")!=std::string::npos);
    sensorOk=false;handleCommand("CAM_RECALIBRATE:4456");assert(!g_cam_initialized);
    assert(Serial.tx.find("CAM_RECALIBRATED:4456,ERROR:")!=std::string::npos);
    sensorOk=true;handleCommand("CAM_RECALIBRATE:4457");assert(g_cam_initialized);
    memoryOk=false;handleCommand("PREVIEW:1235");
    assert(Serial.tx.find("CAPTURE_ERROR:1235,preview_memory")!=std::string::npos && !g_inference_running);
    memoryOk=true;captureOk=false;handleCommand("PREVIEW:1236");
    assert(Serial.tx.find("CAPTURE_ERROR:1236,capture")!=std::string::npos && !g_inference_running);
}
'''
        self.compile_run(harness,CAM)

    def test_transport_helpers_are_identical_between_boards(self):
        self.assertEqual((MAIN/'camera_transport.h').read_bytes(),(CAM/'camera_transport.h').read_bytes())


if __name__=='__main__':
    unittest.main()
