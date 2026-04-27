#include "blob_detector.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifndef PI_BUBBLE_PI
#define PI_BUBBLE_PI 3.14159265358979323846
#endif

static int is_foreground(const uint8_t* binary, int index) {
    return binary[index] > 0;
}

int bubble_detect_from_binary(
    const uint8_t* binary,
    int width,
    int height,
    int min_area,
    int max_area,
    float min_circularity,
    BubbleBlob* out_blobs,
    int max_blobs
) {
    if (binary == NULL || out_blobs == NULL || width <= 0 || height <= 0 || max_blobs <= 0) {
        return -1;
    }

    const int total = width * height;
    uint8_t* visited = (uint8_t*)calloc((size_t)total, sizeof(uint8_t));
    int* queue = (int*)malloc((size_t)total * sizeof(int));
    if (visited == NULL || queue == NULL) {
        free(visited);
        free(queue);
        return -1;
    }

    const int offsets_x[4] = {1, -1, 0, 0};
    const int offsets_y[4] = {0, 0, 1, -1};
    int found = 0;

    memset(out_blobs, 0, (size_t)max_blobs * sizeof(BubbleBlob));

    for (int start = 0; start < total; ++start) {
        if (visited[start] || !is_foreground(binary, start)) {
            continue;
        }

        visited[start] = 1;
        int head = 0;
        int tail = 0;
        queue[tail++] = start;

        int area = 0;
        int perimeter = 0;
        int left = width;
        int top = height;
        int right = 0;
        int bottom = 0;
        uint64_t sum_x = 0;
        uint64_t sum_y = 0;

        while (head < tail) {
            const int index = queue[head++];
            const int y = index / width;
            const int x = index - (y * width);

            area += 1;
            sum_x += (uint64_t)x;
            sum_y += (uint64_t)y;

            if (x < left) {
                left = x;
            }
            if (x > right) {
                right = x;
            }
            if (y < top) {
                top = y;
            }
            if (y > bottom) {
                bottom = y;
            }

            for (int neighbor = 0; neighbor < 4; ++neighbor) {
                const int nx = x + offsets_x[neighbor];
                const int ny = y + offsets_y[neighbor];

                if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                    perimeter += 1;
                    continue;
                }

                const int nindex = ny * width + nx;
                if (!is_foreground(binary, nindex)) {
                    perimeter += 1;
                    continue;
                }

                if (!visited[nindex]) {
                    visited[nindex] = 1;
                    queue[tail++] = nindex;
                }
            }
        }

        if (area < min_area || area > max_area || perimeter <= 0) {
            continue;
        }

        const float circularity =
            (float)(4.0 * PI_BUBBLE_PI * (double)area / ((double)perimeter * (double)perimeter));
        if (circularity < min_circularity) {
            continue;
        }

        if (found < max_blobs) {
            BubbleBlob* blob = &out_blobs[found];
            blob->x = (int)(sum_x / (uint64_t)area);
            blob->y = (int)(sum_y / (uint64_t)area);
            blob->area = area;
            blob->perimeter = perimeter;
            blob->circularity = circularity;
            blob->left = left;
            blob->top = top;
            blob->right = right;
            blob->bottom = bottom;
        }
        found += 1;
    }

    free(visited);
    free(queue);
    return found < max_blobs ? found : max_blobs;
}
