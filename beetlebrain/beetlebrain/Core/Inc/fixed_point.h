/*
 * fixed_point.h
 *
 *  Created on: Apr 9, 2024
 *      Author: pauli
 */

#ifndef INC_FIXED_POINT_H_
#define INC_FIXED_POINT_H_


#define INT_TO_FIXED_POINT(value, shift_amount) ((value) << shift_amount)
#define FLOAT_TO_FIXED_POINT(value, shift_amount) ((value * (1 << shift_amount)))
#define FIXED_POINT_MUL(value1, value2, shift_amount) ((value1 * value2) >> shift_amount)
#define FIXED_POINT_TO_INT(value, shift_amount) ((value) >> shift_amount)


typedef int32_t Fixed32_11;
#define FP32_11_INT_TO_FP(value) INT_TO_FIXED_POINT(value, 11)
#define FP32_11_FLOAT_TO_FP(value) ((Fixed32_11) FLOAT_TO_FIXED_POINT(value, 11))
#define FP32_11_MUL(value1, value2) FIXED_POINT_MUL(value1, value2, 11)
#define FP32_11_FP_TO_INT(value) FIXED_POINT_TO_INT(value, 11)


typedef int32_t Fixed32_19;
#define FP32_19_INT_TO_FP(value) INT_TO_FIXED_POINT(value, 19)
#define FP32_19_FLOAT_TO_FP(value) ((Fixed32_11) FLOAT_TO_FIXED_POINT(value, 19))
#define FP32_19_MUL(value1, value2) FIXED_POINT_MUL(value1, value2, 19)
#define FP32_19_FP_TO_INT(value) FIXED_POINT_TO_INT(value, 19)

#endif /* INC_FIXED_POINT_H_ */
