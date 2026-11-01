#ifndef ENCODER_H
#define ENCODER_H
#include <stdint.h>
typedef struct { uint32_t left, right; } EncoderCounts;
extern volatile uint32_t g_encoder_left_edges, g_encoder_right_edges;
void Encoder_Init(void);
EncoderCounts Encoder_Read(void);
/* A-only unsigned edge counts: no inferred reverse direction. */
void Encoder_LeftEdge(void);
void Encoder_RightEdge(void);
#endif
