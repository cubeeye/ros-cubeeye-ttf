/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef VOXEL_DENOISE_H
#define VOXEL_DENOISE_H

#include "Filter.h"

#define _MATH_DEFINES
#include <math.h>

namespace Voxel
{
  
/**
 * \addtogroup Flt
 * @{
 */
  
class VOXEL_EXPORT DenoiseFilter: public Filter
{
protected:
	float _thr;
	uint _nAmplitude;
	//uint _nPhaseStd;
	uint _nMinValue;
	uint _nMode;
    uint _nX1, _nX2, _nY1, _nY2;

	FrameSize _size;

	virtual void _onSet(const FilterParameterPtr &f);

	template <typename T>
	bool _filter(const T *in, const T *a, T *out);

	template <typename T>
	bool _filter2(const T *in, const T *a, T *out);

	template <typename T>
	bool _filter3(const T *in, const T *a, T *out);

	virtual bool _filter(const FramePtr &in, FramePtr &out);
  
  
public:
    DenoiseFilter(float thr = 1500.0f, uint nAmplitude = 30, uint nMinValue = 80, uint nMode = 0, uint nX1=0, uint nX2=0, uint nY1=0, uint nY2=0);
  
   virtual void reset();
  
   virtual ~DenoiseFilter() {}
};
/**
 * @}
 */

}
#endif // VOXEL_DENOISE_H
