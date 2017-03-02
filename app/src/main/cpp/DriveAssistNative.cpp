#include <jni.h>
#include <android/native_window.h>
#include <android/native_window_jni.h>

#include "DrivingAssistant.h"

extern "C"
jlong
Java_riteh_driveassist_DrivingAssistant_nativeCreateDrivingAssistant(
        JNIEnv *env,
        jobject /*thisObject*/
) {
    jlong drivingAssistantAddress = (jlong) new DrivingAssistant(0.1, 0.6, 0.5, 1.0);
    return drivingAssistantAddress;
}

extern "C"
void
Java_riteh_driveassist_DrivingAssistant_nativeDestroyDrivingAssistant(
        JNIEnv *env,
        jobject /*thisObject*/,
        jlong drivingAssistantAddress
) {
    delete (DrivingAssistant *) drivingAssistantAddress;
}

extern "C"
void
Java_riteh_driveassist_DrivingAssistant_nativeUpdate(
        JNIEnv *env,
        jobject /*thisObject*/,
        jlong drivingAssistantAddress,
        jlong inputFrameAddress,
        jbooleanArray outIsLaneDepartedAddress,
        jbooleanArray outIsRedLightDetectedAddress
) {
    DrivingAssistant *drivingAssistant = (DrivingAssistant *) drivingAssistantAddress;
    cv::Mat& inputFrame = *(cv::Mat*) inputFrameAddress;

    drivingAssistant->update(inputFrame);

    jboolean *outIsLaneDeparted = env->GetBooleanArrayElements(outIsLaneDepartedAddress, NULL);
    jboolean *outIsRedLightDetected = env->GetBooleanArrayElements(outIsRedLightDetectedAddress, NULL);

    outIsLaneDeparted[0] = drivingAssistant->isLaneDeparted();
    outIsRedLightDetected[0] = drivingAssistant->isRedLightDetected();

    env->ReleaseBooleanArrayElements(outIsLaneDepartedAddress, outIsLaneDeparted, 0);
    env->ReleaseBooleanArrayElements(outIsRedLightDetectedAddress, outIsRedLightDetected, 0);

}