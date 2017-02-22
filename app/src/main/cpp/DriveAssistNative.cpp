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
        jobject frameByteArrayAddress,
        jfloatArray intersectionsArrayAddress,
        jfloatArray slopesArrayAddress) {
/* This works... Make sure you enc->Release..
    unsigned char* frameData = (unsigned char*) env->GetByteArrayElements(frameByteArrayAddr, NULL);
*/
    float *intersectionsArray = env->GetFloatArrayElements(intersectionsArrayAddress, NULL);
    float *slopesArray = env->GetFloatArrayElements(slopesArrayAddress, NULL);
    unsigned char* frameData = (unsigned char*) env->GetDirectBufferAddress(frameByteArrayAddress);
    //float *intersectionsArray = (float *) env->GetDirectBufferAddress(intersectionsArrayAddress);
    //float *slopesArray = (float *) env->GetDirectBufferAddress(slopesArrayAddress);

    Mat frame(frameHeight, frameWidth, CV_8UC1, frameData);

    Vec2f intersections;
    Vec2f slopes;

    bool lanesFound = da::lane_intersections(frame, intersections, slopes);

    intersectionsArray[0] = intersections[0];
    intersectionsArray[1] = intersections[1];
    slopesArray[0] = slopes[0];
    slopesArray[1] = slopes[1];

    jboolean returnValue;
    if (lanesFound) returnValue = JNI_TRUE;
    else returnValue = JNI_FALSE;

    env->ReleaseFloatArrayElements(intersectionsArrayAddress, intersectionsArray, 0);
    env->ReleaseFloatArrayElements(slopesArrayAddress, slopesArray, 0);
    //env->ReleaseByteArrayElements(frameByteArrayAddr, (jbyte*) frameData, JNI_ABORT);

    return returnValue;
}
