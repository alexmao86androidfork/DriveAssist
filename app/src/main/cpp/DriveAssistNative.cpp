#include <jni.h>
//#include <opencv2/core/core.hpp>
#include "DriveAssist.h"

extern "C"
jboolean
Java_riteh_driveassist_MainActivity_laneIntersections(
        JNIEnv *env,
        jobject /*thisObject*/,
        jint frameWidth,
        jint frameHeight,
        jbyteArray frameByteArrayAddr,
        jfloatArray outputIntersectionAddr,
        jfloatArray outputSlopesAddr) {

    unsigned char* frameData = (unsigned char*) env->GetByteArrayElements(frameByteArrayAddr, NULL);
    float *intersectionArray = env->GetFloatArrayElements(outputIntersectionAddr, NULL);
    float *slopeArray = env->GetFloatArrayElements(outputSlopesAddr, NULL);

    Mat frame(frameHeight, frameWidth, CV_8UC1, frameData);

    Vec2f intersections;
    Vec2f slopes;

    bool lanesFound = da::lane_intersections(frame, intersections, slopes);

    intersectionArray[0] = intersections[0];
    intersectionArray[1] = intersections[1];
    slopeArray[0] = slopes[0];
    slopeArray[1] = slopes[1];

    jboolean returnValue;
    if (lanesFound) returnValue = JNI_TRUE;
    else returnValue = JNI_FALSE;

    env->ReleaseFloatArrayElements(outputIntersectionAddr, intersectionArray, 0);
    env->ReleaseFloatArrayElements(outputSlopesAddr, slopeArray, 0);
    env->ReleaseByteArrayElements(frameByteArrayAddr, (jbyte*) frameData, JNI_ABORT);

    return returnValue;
}
