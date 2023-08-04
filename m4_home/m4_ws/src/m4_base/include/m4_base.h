#include <vector>
#include <string>
#include "DynamixelInterface.h"

class M4Base
{
	public:
		M4Base();

		// Transformations
		void stand();
		void crouch();
		void sit();
		void uav();
		void mip();

	private:
		DynamixelInterface dxl;
		
};