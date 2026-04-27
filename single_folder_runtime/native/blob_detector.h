#ifndef PI_BUBBLE_BLOB_DETECTOR_H
#define PI_BUBBLE_BLOB_DETECTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct BubbleBlob {
    int x;
    int y;
    int area;
    int perimeter;
    float circularity;
    int left;
    int top;
    int right;
    int bottom;
} BubbleBlob;

int bubble_detect_from_binary(
    const uint8_t* binary,
    int width,
    int height,
    int min_area,
    int max_area,
    float min_circularity,
    BubbleBlob* out_blobs,
    int max_blobs
);

#ifdef __cplusplus
}
#endif

#endif
