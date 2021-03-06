// AUTOGENERATED FILE - DO NOT MODIFY!
// This file generated by Djinni from mynteye_types.djinni

#pragma once

#include <cstdint>
#include <memory>

namespace mynteye_jni {

class Frame;
struct ImgData;

/** Device stream data */
class StreamData {
public:
    virtual ~StreamData() {}

    virtual ImgData Img() = 0;

    virtual std::shared_ptr<::mynteye_jni::Frame> Frame() = 0;

    virtual int64_t FrameId() = 0;
};

}  // namespace mynteye_jni
