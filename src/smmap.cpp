#include "smmap.h"

namespace LMapping {

const PointAccumulator& PointAccumulator::Unknown(){
	if (! unknown_ptr)
		unknown_ptr=new PointAccumulator;
	return *unknown_ptr;
}

PointAccumulator* PointAccumulator::unknown_ptr=0;

};


