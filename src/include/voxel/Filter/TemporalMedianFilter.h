/*
* TI Voxel Lib component.
*
* Copyright (c) 2014 Texas Instruments Inc.
*/

#ifndef VOXEL_TEMPORAL_MEDIAN_FILTER_H
#define VOXEL_TEMPORAL_MEDIAN_FILTER_H

#include "Filter.h"

#include <string.h>

namespace Voxel
{
	/**
	* \addtogroup Flt
	* @{
	*/
	class TemporalMedianFilter : public Filter
	{
	protected:
		float _fFilterSize;
		float _fEps;
		uint   _nMode;

		FrameSize _size;
		List<Vector<ByteType>> _history;
		Vector<ByteType> _current;

		virtual void _onSet(const FilterParameterPtr &f);

		template <typename T>
		void _getMedian(IndexType offset, T &value);

		template <typename T>
		bool _filter(const T *in, T *out);

		virtual bool _filter(const FramePtr &in, FramePtr &out);

	public:
		TemporalMedianFilter(float fFilterSize = 3.0f, float fEps = 2500.0f, uint nMode = 1);
		virtual ~TemporalMedianFilter() {}

		virtual void reset();
	};

	/**
	* @}
	*/

}
#endif // VOXEL_TEMPORAL_MEDIAN_FILTER_H
