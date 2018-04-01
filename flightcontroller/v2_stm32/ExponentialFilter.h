#ifndef _EXPONENTIALFILTER_H_
#define _EXPONENTIALFILTER_H_
#endif
#pragma once

/* 
* Implements a simple linear recursive exponential filter. 
* See: http://www.statistics.com/glossary&term_id=756 */
// Specialization for floating point math. 
class ExponentialFilter {
	public:

		float filter(float newValue) {
    		current = weight * newValue + (1.0 - weight) * current;
			return current;
		}

  		void setWeight(float newWeight) {
    		weight = newWeight/100.0;
  		}

		float getWeight() const { 
			return weight * 100.0;
		}

		void setCurrent(float newCurrent) {
			current = newCurrent;
		}

		float getCurrent() {
			return current;
		}
	
	private:
		float weight;
		float current;

};
