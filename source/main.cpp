#include <chrono>
#include <thread>

#include "renderer.h"
#include "timeprofiler.h"
#include "rigidbody.h"


using namespace RenderingFramework3D;
using namespace PhysicsSim;
using namespace MathUtil;


static const float maxCamRotXCos = std::cos(PI/2);

static bool ProcessInput(Camera& cam,  Window& window);

int main(int argc, char** args) {
    printf("Physics Simulation\n");

    unsigned windowWidth=1240, windowHeight=720;

//  Create Window
    Window window;
    if(window.Initialize(false, windowWidth, windowHeight , "Physics Simulation") == false) {
        printf("failed to initialize window\n");
        return -1;
    }
    window.SetMouseVisibility(false);

//  Create Renderer
    Renderer renderer;
    if(renderer.Initialize(window)==false) {
        printf("failed to initialize renderer\n");
        return -1; 
    }

//  set default global uniform shader inputs including directional and ambient lights
    renderer.SetLightDirection(Vec<3>({-1,-1, 1}));
    renderer.SetLightColour(Vec<4>({1,1,1,1}));
    renderer.SetLightIntensity(1);
    renderer.SetAmbientLightIntensity(0.06);

//  Create Camera
    Camera mainCamera({ 0,0,windowWidth, windowHeight });
    mainCamera.Move(Vec<3>({0,0,-400}));
//  mainCamera.Rotate(mainCamera.GetCameraAxisX(), PI/4);

//  Cube Mesh for rigidbodies
    Mesh cubeMesh = Mesh::Cube(renderer);
    cubeMesh.LoadMesh();

    RigidBodyCuboid obj1(cubeMesh, 50, 100, 10, 200);
    obj1.GetMaterial().colour = Vec<4>({1,1,1,1});
    obj1.SetPosition(Vec<3> ({0,0,0}));
    //obj1.Rotate(Vec<3>({0,0,1}), PI/4);
    obj1.SetAngularVelocity(Vec<3>({2,0.001,0.001}));

    TimeProfiler timer;
    unsigned n = 1000;
    timer.Start();
    int i=0;
    while(true) {
        obj1.Step(5);

    //  rendering
        renderer.DrawObject(obj1, mainCamera);        
        renderer.PresentFrame();
    
    //  Report simulation status after "n" frames
        i %= n;
        if (i == 0) {
            timer.Check("Frame Time", n);
            timer.Start();
            printf("Angular Momentum: ");
            obj1.GetAngularMomentum().Print();
            
            printf("Angular Velocity: ");
            obj1.GetAngularVelocity().Print();
            printf("\n");
        }
        i++;
    
    //  limit CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

    //  process inputs: camera movement
        if(ProcessInput(mainCamera,window)==false) {
            break;
        }
    }

    renderer.Cleanup();
    window.Cleanup();

    return 0;
}


bool ProcessInput(Camera& cam,  Window& window) {
    static bool pointerMode=false;
    auto disp = window.GetMouseDisplacement();

    if (window.IsResized()) {
        cam.SetViewPort({ 0,0,window.GetWidth(), window.GetHeight() });
    }

    float scale = 0.01;
    if (window.CheckKeyPressEvent(Window::KEY_LCTRL)) {
        pointerMode = !pointerMode;
        window.SetMouseVisibility(pointerMode);
    }
    if (pointerMode==false || pointerMode && window.IsMouseButtonPressed(Window::MOUSE_RIGHT)) {
        if(fabs(disp(0)) > 0.001 && fabs(disp(0)) < 100) {
            cam.Rotate(Vec<3>({0,1,0}), disp(0) * scale);
        }
        if(fabs(disp(1)) > 0.001 && fabs(disp(1)) < 100) {
            cam.Rotate(cam.GetCameraAxisX(), disp(1) * scale);
            if(cam.GetCameraAxisY().Dot(Vec<3>({0,1,0})) < maxCamRotXCos) {
                cam.Rotate(cam.GetCameraAxisX(), -disp(1) * scale);
            }
        }
    }
    scale = 0.8;
    if (window.IsKeyPressed(Window::KEY_W)) {
        cam.Move(scale * cam.GetCameraAxisZ());
    }
    if (window.IsKeyPressed(Window::KEY_S)) {
        cam.Move(-scale * cam.GetCameraAxisZ());
    }
    if (window.IsKeyPressed(Window::KEY_A)) {
        cam.Move(-scale * cam.GetCameraAxisX());
    }
    if (window.IsKeyPressed(Window::KEY_D)) {
        cam.Move(scale * cam.GetCameraAxisX());
    }
    if (window.IsKeyPressed(Window::KEY_SPACE)) {
        cam.Move(scale * cam.GetCameraAxisY());
    }
    if (window.IsKeyPressed(Window::KEY_LSHIFT)) {
        cam.Move(-scale * cam.GetCameraAxisY() );
    }

    window.Update();
    if(window.CheckExit()) {
        return false;
    }
    return true;
}