#include <jni.h>
#include "DriveAssist.h"

extern "C"
bool
Java_riteh_driveassist_MainActivity_laneIntersections(
        JNIEnv *env,
        jobject thisObject,
        Mat frame,
        Vec2f &intersections,
        Vec2f &slopes) {

    return da::lane_intersections(frame, intersections, slopes);
}
