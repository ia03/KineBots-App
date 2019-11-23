#include <jni.h>


extern "C"
JNIEXPORT jstring JNICALL
Java_cvlab_example_com_MainActivity_jni_1test(JNIEnv *env, jobject thiz) {
    // TODO: implement jni_test()
    return env->NewStringUTF("Hello From JNI");
}