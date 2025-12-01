#include <iostream>
#include <memory>
#include <chrono>
#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>
#include <opencv2/opencv.hpp>
#include <sys/mman.h>

using namespace libcamera;
using namespace std;
using namespace cv;

class RpiCameraStream {
private:
    unique_ptr<CameraManager> cm;
    shared_ptr<Camera> camera;
    unique_ptr<FrameBufferAllocator> allocator;
    unique_ptr<CameraConfiguration> config;
    Stream *stream;
    vector<unique_ptr<Request>> requests;
    bool streaming;
    int width, height;
    
public:
    RpiCameraStream() : stream(nullptr), streaming(false), width(640), height(480) {}
    
    ~RpiCameraStream() {
        stopStream();
        if (camera) {
            camera->release();
        }
        if (cm) {
            cm->stop();
        }
    }
    
    int initialize() {
        cm = make_unique<CameraManager>();
        int ret = cm->start();
        if (ret) {
            cerr << "Failed to start camera manager" << endl;
            return ret;
        }
        
        if (cm->cameras().empty()) {
            cerr << "No cameras detected" << endl;
            return -1;
        }
        
        string cameraId = cm->cameras()[0]->id();
        camera = cm->get(cameraId);
        
        if (camera->acquire()) {
            cerr << "Failed to acquire camera" << endl;
            return -1;
        }
        
        cout << "Camera acquired: " << cameraId << endl;
        return 0;
    }
    
    int configure(int w = 640, int h = 480) {
        width = w;
        height = h;
        
        config = camera->generateConfiguration({StreamRole::VideoRecording});
        if (!config) {
            cerr << "Failed to generate configuration" << endl;
            return -1;
        }
        
        StreamConfiguration &streamConfig = config->at(0);
        streamConfig.size.width = width;
        streamConfig.size.height = height;
        streamConfig.pixelFormat = PixelFormat::fromString("RGB888");
        
        CameraConfiguration::Status validation = config->validate();
        if (validation == CameraConfiguration::Invalid) {
            cerr << "Camera configuration invalid" << endl;
            return -1;
        }
        
        if (camera->configure(config.get())) {
            cerr << "Failed to configure camera" << endl;
            return -1;
        }
        
        stream = streamConfig.stream();
        
        // Update actual dimensions
        width = streamConfig.size.width;
        height = streamConfig.size.height;
        
        cout << "Camera configured: " << width << "x" << height << endl;
        
        allocator = make_unique<FrameBufferAllocator>(camera);
        if (allocator->allocate(stream) < 0) {
            cerr << "Failed to allocate buffers" << endl;
            return -1;
        }
        
        return 0;
    }
    
    int startStream() {
        // Create requests for each buffer
        for (const unique_ptr<FrameBuffer> &buffer : allocator->buffers(stream)) {
            unique_ptr<Request> request = camera->createRequest();
            if (!request) {
                cerr << "Failed to create request" << endl;
                return -1;
            }
            
            if (request->addBuffer(stream, buffer.get())) {
                cerr << "Failed to add buffer to request" << endl;
                return -1;
            }
            
            requests.push_back(move(request));
        }
        
        // Set up request completion callback
        camera->requestCompleted.connect(this, &RpiCameraStream::requestComplete);
        
        if (camera->start()) {
            cerr << "Failed to start camera" << endl;
            return -1;
        }
        
        // Queue all requests
        for (unique_ptr<Request> &request : requests) {
            camera->queueRequest(request.get());
        }
        
        streaming = true;
        cout << "Camera streaming started" << endl;
        return 0;
    }
    
    void stopStream() {
        if (streaming) {
            camera->stop();
            streaming = false;
            cout << "Camera streaming stopped" << endl;
        }
    }
    
    void displayStream() {
        namedWindow("Raspberry Pi Camera", WINDOW_AUTOSIZE);
        
        cout << "Press 'q' to quit" << endl;
        
        while (streaming) {
            // The actual frame processing happens in requestComplete callback
            // This loop just checks for quit key
            int key = waitKey(30);
            if (key == 'q' || key == 'Q' || key == 27) { // 'q' or ESC
                break;
            }
        }
        
        destroyAllWindows();
    }
    
private:
    void requestComplete(Request *request) {
        if (request->status() == Request::RequestCancelled) {
            return;
        }
        
        // Get the buffer from the request
        const Request::BufferMap &buffers = request->buffers();
        const FrameBuffer *buffer = buffers.begin()->second;
        
        // Convert buffer to OpenCV Mat and display
        Mat frame = bufferToMat(buffer);
        if (!frame.empty()) {
            imshow("Raspberry Pi Camera", frame);
        }
        
        // Requeue the request for continuous capture
        request->reuse(Request::ReuseBuffers);
        camera->queueRequest(request);
    }
    
    Mat bufferToMat(const FrameBuffer *buffer) {
        const FrameBuffer::Plane &plane = buffer->planes()[0];
        
        // Memory map the buffer
        void *mem = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, 
                        plane.fd.get(), 0);
        if (mem == MAP_FAILED) {
            cerr << "Failed to mmap buffer" << endl;
            return Mat();
        }
        
        // Create OpenCV Mat from the buffer (RGB format)
        Mat frame(height, width, CV_8UC3, mem);
        
        // Convert RGB to BGR (OpenCV uses BGR)
        Mat bgrFrame;
        cvtColor(frame, bgrFrame, COLOR_RGB2BGR);
        
        // Clone the frame so we can unmap the buffer
        Mat result = bgrFrame.clone();
        
        munmap(mem, plane.length);
        
        return result;
    }
};

int main(int argc, char *argv[]) {
    RpiCameraStream camera;
    
    // Initialize camera
    if (camera.initialize() != 0) {
        cerr << "Failed to initialize camera" << endl;
        return -1;
    }
    
    // Configure camera (640x480 by default, you can change this)
    int width = 640;
    int height = 480;
    
    if (argc >= 3) {
        width = atoi(argv[1]);
        height = atoi(argv[2]);
    }
    
    if (camera.configure(width, height) != 0) {
        cerr << "Failed to configure camera" << endl;
        return -1;
    }
    
    // Start streaming
    if (camera.startStream() != 0) {
        cerr << "Failed to start stream" << endl;
        return -1;
    }
    
    // Display the video stream
    camera.displayStream();
    
    cout << "Stream closed" << endl;
    return 0;
}