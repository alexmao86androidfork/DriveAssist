#include <jni.h>
#include <android/native_window.h>
#include <android/native_window_jni.h>

//#include <opencv2/core/core.hpp>
#include "DriveAssist.h"

extern "C"
jboolean
Java_riteh_driveassist_MainActivity_laneIntersections(
        JNIEnv *env,
        jobject /*thisObject*/,
        jlong inputFrameAddress,
        jfloatArray intersectionsArrayAddress,
        jfloatArray slopesArrayAddress
) {
    Mat& inputFrame = *(Mat*) inputFrameAddress;
    float *intersectionsArray = env->GetFloatArrayElements(intersectionsArrayAddress, NULL);
    float *slopesArray = env->GetFloatArrayElements(slopesArrayAddress, NULL);
    //unsigned char* YPlane = (unsigned char*) env->GetDirectBufferAddress(YPlaneBuffer);

    Vec2f intersections;
    Vec2f slopes;

    bool lanesFound = da::lane_intersections(inputFrame, intersections, slopes);

    intersectionsArray[0] = intersections[0];
    intersectionsArray[1] = intersections[1];
    slopesArray[0] = slopes[0];
    slopesArray[1] = slopes[1];

    jboolean returnValue;
    if (lanesFound) returnValue = JNI_TRUE;
    else returnValue = JNI_FALSE;

    env->ReleaseFloatArrayElements(intersectionsArrayAddress, intersectionsArray, 0);
    env->ReleaseFloatArrayElements(slopesArrayAddress, slopesArray, 0);

    return returnValue;
}
