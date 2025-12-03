#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <SDL2/SDL.h>
#include <cstring>

using namespace std;

class V4L2Camera {
private:
    int fd;
    struct v4l2_buffer bufferinfo;
    void *buffer_start;
    size_t buffer_length;
    int width, height;
    
public:
    V4L2Camera() : fd(-1), buffer_start(nullptr), buffer_length(0) {}
    
    ~V4L2Camera() {
        if (buffer_start) {
            munmap(buffer_start, buffer_length);
        }
        if (fd >= 0) {
            close(fd);
        }
    }
    
    int open(const char* device, int w, int h) {
        width = w;
        height = h;
        
        // Open the device
        fd = ::open(device, O_RDWR);
        if (fd < 0) {
            cerr << "Failed to open device: " << device << endl;
            return -1;
        }
        
        // Set format
        struct v4l2_format format;
        memset(&format, 0, sizeof(format));
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = width;
        format.fmt.pix.height = height;
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        format.fmt.pix.field = V4L2_FIELD_NONE;
        
        if (ioctl(fd, VIDIOC_S_FMT, &format) < 0) {
            cerr << "Failed to set format" << endl;
            return -1;
        }
        
        // Get actual format
        width = format.fmt.pix.width;
        height = format.fmt.pix.height;
        
        cout << "Camera format set to: " << width << "x" << height << endl;
        
        // Request buffer
        struct v4l2_requestbuffers req;
        memset(&req, 0, sizeof(req));
        req.count = 1;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        
        if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
            cerr << "Failed to request buffer" << endl;
            return -1;
        }
        
        // Query buffer
        memset(&bufferinfo, 0, sizeof(bufferinfo));
        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        bufferinfo.index = 0;
        
        if (ioctl(fd, VIDIOC_QUERYBUF, &bufferinfo) < 0) {
            cerr << "Failed to query buffer" << endl;
            return -1;
        }
        
        // Map buffer
        buffer_start = mmap(NULL, bufferinfo.length, PROT_READ | PROT_WRITE,
                           MAP_SHARED, fd, bufferinfo.m.offset);
        buffer_length = bufferinfo.length;
        
        if (buffer_start == MAP_FAILED) {
            cerr << "Failed to map buffer" << endl;
            return -1;
        }
        
        return 0;
    }
    
    int startCapture() {
        // Queue the buffer
        if (ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0) {
            cerr << "Failed to queue buffer" << endl;
            return -1;
        }
        
        // Start streaming
        int type = bufferinfo.type;
        if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
            cerr << "Failed to start streaming" << endl;
            return -1;
        }
        
        cout << "Camera streaming started" << endl;
        return 0;
    }
    
    void* captureFrame() {
        // Dequeue buffer
        if (ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0) {
            cerr << "Failed to dequeue buffer" << endl;
            return nullptr;
        }
        
        return buffer_start;
    }
    
    void releaseFrame() {
        // Requeue buffer
        if (ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0) {
            cerr << "Failed to requeue buffer" << endl;
        }
    }
    
    int getWidth() const { return width; }
    int getHeight() const { return height; }
};

// Convert YUYV to RGB
void yuyvToRgb(const unsigned char* yuyv, unsigned char* rgb, int width, int height) {
    for (int i = 0; i < width * height * 2; i += 4) {
        int y1 = yuyv[i];
        int u = yuyv[i + 1];
        int y2 = yuyv[i + 2];
        int v = yuyv[i + 3];
        
        // Convert to RGB (first pixel)
        int c1 = y1 - 16;
        int c2 = y2 - 16;
        int d = u - 128;
        int e = v - 128;
        
        int r1 = (298 * c1 + 409 * e + 128) >> 8;
        int g1 = (298 * c1 - 100 * d - 208 * e + 128) >> 8;
        int b1 = (298 * c1 + 516 * d + 128) >> 8;
        
        int r2 = (298 * c2 + 409 * e + 128) >> 8;
        int g2 = (298 * c2 - 100 * d - 208 * e + 128) >> 8;
        int b2 = (298 * c2 + 516 * d + 128) >> 8;
        
        // Clamp values
        r1 = r1 < 0 ? 0 : (r1 > 255 ? 255 : r1);
        g1 = g1 < 0 ? 0 : (g1 > 255 ? 255 : g1);
        b1 = b1 < 0 ? 0 : (b1 > 255 ? 255 : b1);
        r2 = r2 < 0 ? 0 : (r2 > 255 ? 255 : r2);
        g2 = g2 < 0 ? 0 : (g2 > 255 ? 255 : g2);
        b2 = b2 < 0 ? 0 : (b2 > 255 ? 255 : b2);
        
        int pixelIndex = (i / 2) * 3;
        rgb[pixelIndex] = r1;
        rgb[pixelIndex + 1] = g1;
        rgb[pixelIndex + 2] = b1;
        rgb[pixelIndex + 3] = r2;
        rgb[pixelIndex + 4] = g2;
        rgb[pixelIndex + 5] = b2;
    }
}

int main(int argc, char* argv[]) {
    const char* device = "/dev/video0";
    int width = 640;
    int height = 480;
    
    if (argc >= 3) {
        width = atoi(argv[1]);
        height = atoi(argv[2]);
    }
    
    // Initialize V4L2 camera
    V4L2Camera camera;
    if (camera.open(device, width, height) < 0) {
        return -1;
    }
    
    width = camera.getWidth();
    height = camera.getHeight();
    
    if (camera.startCapture() < 0) {
        return -1;
    }
    
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        cerr << "SDL initialization failed: " << SDL_GetError() << endl;
        return -1;
    }
    
    // Create window
    SDL_Window* window = SDL_CreateWindow("Raspberry Pi Camera",
                                          SDL_WINDOWPOS_CENTERED,
                                          SDL_WINDOWPOS_CENTERED,
                                          width, height,
                                          SDL_WINDOW_SHOWN);
    if (!window) {
        cerr << "Window creation failed: " << SDL_GetError() << endl;
        SDL_Quit();
        return -1;
    }
    
    // Create renderer
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        cerr << "Renderer creation failed: " << SDL_GetError() << endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }
    
    // Create texture
    SDL_Texture* texture = SDL_CreateTexture(renderer,
                                            SDL_PIXELFORMAT_RGB24,
                                            SDL_TEXTUREACCESS_STREAMING,
                                            width, height);
    if (!texture) {
        cerr << "Texture creation failed: " << SDL_GetError() << endl;
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }
    
    cout << "Press ESC or close window to quit" << endl;
    
    // RGB buffer
    unsigned char* rgbBuffer = new unsigned char[width * height * 3];
    
    // Main loop
    bool running = true;
    SDL_Event event;
    
    while (running) {
        // Handle events
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            }
            else if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running = false;
                }
            }
        }
        
        // Capture frame
        void* frame = camera.captureFrame();
        if (frame) {
            // Convert YUYV to RGB
            yuyvToRgb((unsigned char*)frame, rgbBuffer, width, height);
            
            // Update texture
            SDL_UpdateTexture(texture, NULL, rgbBuffer, width * 3);
            
            // Render
            SDL_RenderClear(renderer);
            SDL_RenderCopy(renderer, texture, NULL, NULL);
            SDL_RenderPresent(renderer);
            
            // Release frame
            camera.releaseFrame();
        }
        
        SDL_Delay(10); // ~100 FPS max
    }
    
    // Cleanup
    delete[] rgbBuffer;
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    
    cout << "Camera closed" << endl;
    return 0;
}
